#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自主搜索编排：协调「未知区域探索」与「目标检测/接近」。

业务目的：
  检测只在目标进入相机视野时有效，而救援恰恰是不知道人在哪。本节点补上
  「目标不在视野时怎么办」：让机器人按 frontier 主动探索未知区域，边建图
  边搜索；一旦发现目标就暂停探索，把底盘控制权让给 FollowTarget。

  m-explore-ros2（explore_lite）只负责探索，不知道检测的存在；target_fusion
  只负责目标决策，不知道探索的存在。本节点是把两者粘起来的编排层：

    /rescue/fusion_state ─> search_coordinator ─> /explore/resume
         (idle/pending/following)                  (true=探索 / false=暂停)

为什么必须自己写这层：
  explore_lite 会持续给 Nav2 下发导航目标；如果检测到目标后不暂停它，
  两路目标会互相抢占 Nav2，表现为机器人在「去目标」和「去 frontier」之间
  反复横跳。这个协同逻辑没有现成实现。

状态机：
  IDLE        未启用搜索（默认），不发任何指令
  SEARCHING   正在探索未知区域
  YIELDED     已发现目标，暂停探索让位给 FollowTarget
  目标结束（fusion_state 回到 idle）并等待 resume_delay 后自动回到 SEARCHING

安全边界：
  本节点不发布速度指令，只发布 Bool 控制探索开关；运动始终由 Nav2 独占。
  探索期间仍受 twist_mux 与 Nav2 避障约束。
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class SearchCoordinatorNode(Node):
    """探索与目标接近的编排节点。"""

    # 状态取值
    STATE_IDLE = "idle"
    STATE_SEARCHING = "searching"
    STATE_YIELDED = "yielded"

    def __init__(self):
        super().__init__("search_coordinator")

        self.declare_parameter("fusion_state_topic", "/rescue/fusion_state")
        self.declare_parameter("explore_resume_topic", "/explore/resume")
        self.declare_parameter("search_cmd_topic", "/rescue/search_cmd")
        # 默认不启用：自主搜索会让机器人自行移动，属于高风险行为，
        # 必须由操作员显式开启（Web 控制台或命令行）。
        self.declare_parameter("auto_start", False)
        # 目标结束后延迟多久恢复探索（秒）：给机器人留出接近/等待的时间，
        # 也避免目标在丢失边缘反复启停导致抖动。
        self.declare_parameter("resume_delay", 5.0)
        self.declare_parameter("enable", True)

        self._resume_delay = self.get_parameter("resume_delay").value
        self._enabled = self.get_parameter("enable").value

        self._state = self.STATE_IDLE
        self._fusion_state = "idle"     # 来自 target_fusion 的最新状态
        self._last_idle_time = None     # fusion 最近一次回到 idle 的时刻
        self._last_published = None     # 最近一次下发给探索的指令，避免重复发

        self._pub_resume = self.create_publisher(
            Bool, self.get_parameter("explore_resume_topic").value, 10
        )
        self._pub_state = self.create_publisher(
            String, "/rescue/search_state", 10
        )
        self._sub_fusion = self.create_subscription(
            String,
            self.get_parameter("fusion_state_topic").value,
            self._on_fusion_state,
            10,
        )
        # 外部启停搜索（Web 控制台的「搜索模式」按钮）
        self._sub_cmd = self.create_subscription(
            Bool,
            self.get_parameter("search_cmd_topic").value,
            self._on_search_cmd,
            10,
        )

        if self.get_parameter("auto_start").value and self._enabled:
            self._state = self.STATE_SEARCHING
            self.get_logger().warning(
                "auto_start=true，上电即开始自主搜索——确认场地已清空"
            )

        # 1Hz 定时评估状态机：协调是持续行为，不能只靠事件触发
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"搜索编排就绪：初始状态 {self._state}，"
            f"恢复延迟 {self._resume_delay}s，auto_start="
            f"{self.get_parameter('auto_start').value}"
        )

    # ---- 输入 ----------------------------------------------------------

    def _on_fusion_state(self, msg):
        """接收融合节点状态。这是判断「有没有目标」的唯一数据源。"""
        new_state = (msg.data or "").strip()
        if new_state == self._fusion_state:
            return
        self._fusion_state = new_state
        if new_state == "idle":
            # 记录回到空闲的时刻，供 resume_delay 计算
            self._last_idle_time = time.monotonic()
        self.get_logger().info(f"融合状态更新 -> {new_state}")

    def _on_search_cmd(self, msg):
        """外部启停搜索。暂停时立即让探索停下。"""
        if not self._enabled:
            self.get_logger().warning("搜索功能已禁用（enable=false），忽略指令。")
            return
        if msg.data:
            if self._state == self.STATE_IDLE:
                self._state = self.STATE_SEARCHING
                self.get_logger().info("收到指令：开始自主搜索")
        else:
            if self._state != self.STATE_IDLE:
                self._state = self.STATE_IDLE
                self.get_logger().info("收到指令：停止自主搜索")

    # ---- 状态机 --------------------------------------------------------

    def _tick(self):
        """按当前状态与融合状态推进，并下发探索开关指令。"""
        has_target = self._fusion_state in ("pending", "following")

        if self._state == self.STATE_IDLE:
            self._publish_resume(False)
        elif self._state == self.STATE_SEARCHING:
            if has_target:
                # 发现目标：暂停探索，把底盘让给 FollowTarget
                self._state = self.STATE_YIELDED
                self._publish_resume(False)
                self.get_logger().info("发现目标，暂停探索让位给接近动作")
            else:
                self._publish_resume(True)
        elif self._state == self.STATE_YIELDED:
            if has_target:
                self._publish_resume(False)
            else:
                # 目标已结束，等待 resume_delay 后恢复探索
                if self._last_idle_time is None:
                    self._last_idle_time = time.monotonic()
                waited = time.monotonic() - self._last_idle_time
                if waited >= self._resume_delay:
                    self._state = self.STATE_SEARCHING
                    self._publish_resume(True)
                    self.get_logger().info(
                        f"目标已结束并等待 {waited:.1f}s，恢复自主搜索"
                    )
                else:
                    self._publish_resume(False)

        self._publish_state()

    # ---- 输出 ----------------------------------------------------------

    def _publish_resume(self, value):
        """控制探索开关。

        每个 tick 都发而不做去重：explore_lite 可能晚于本节点启动，若只在
        取值变化时发一次，它会错过初始的暂停指令、上电即开始探索，与本节点
        的编排意图相悖。1Hz 的 Bool 消息开销可忽略。
        """
        self._pub_resume.publish(Bool(data=value))
        if self._last_published != value:
            self._last_published = value
            self.get_logger().info(f"探索开关 -> {value}")

    def _publish_state(self):
        self._pub_state.publish(String(data=self._state))


def main(args=None):
    rclpy.init(args=args)
    node = SearchCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前务必停掉探索，避免节点消失后机器人继续自行移动
        node._publish_resume(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
