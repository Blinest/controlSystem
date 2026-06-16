# ==========================================
# 5. 设备选项卡
# ==========================================

# Qt类
import struct
import math
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QGridLayout,
                             QLabel, QComboBox, QDoubleSpinBox, QPushButton, QTabWidget,
                             QFrame, QSplitter, QMessageBox, QGraphicsDropShadowEffect, QAbstractSpinBox, QScrollArea)
from PyQt5.QtCore import Qt, QTimer, pyqtSlot


from Core.serial_worker import SerialWorker
from Core.auth import GlobalHistory
from Core.protocol import ProtocolParser, DataFilter
# 自定义类
from .widgets import AnimatedButton
from Utils.controller import PID
# 工具类
import time
class DeviceTab(QWidget):
    def __init__(self, port_name, parent_logger, auth_service=None, debug_check=None):
        super().__init__()
        self.auth_service = auth_service

        # 循环控制相关
        self.cycle_last_trigger = None   # 'high' 或 'low'
        self.cycle_target_motor_pos = -10.0   # 低阈值时电机要移动到的目标位置（mm）
        # 指令重发检测
        self.last_motor_pos = 0.0           # 上次发送指令时的电机位置
        self.last_sent_target = -10        # 上次发送的目标位置 (mm)
        self.last_send_time = 0             # 上次发送指令的时间戳
        self.pending_retry_target = None   # 当前等待确认的目标位置 (None 表示无等待)
        self.pending_send_time = 0         # 上次发送该指令的时间戳

        # ========== 基本属性 ==========
        self.port_name = port_name
        self.logger = parent_logger
        self.debug_check = debug_check
        self.serial_error = False
        self.start_time = None

        # ========== 串口通信与数据缓冲 ==========
        self.worker = SerialWorker(port_name)
        self.worker.signal_data.connect(self.parse_data)
        self.worker.signal_error.connect(self.handle_serial_error)
        self.recv_buffer = bytearray()

        # ========== 系统运行状态 ==========
        self.is_started = False            # 控制系统是否使能
        self.closed_loop_enabled = False   # 闭环弯曲是否启用
        self.closed_loop_target_angle = 0.0

        # ========== 电机与传感器数量及数据 ==========
        self.num_m = 0
        self.num_s = 0
        self.motor_data = []          # 当前电机数据 [pos, vel, acc]
        self.motor_target = []        # 目标值 [pos, vel, acc]
        self.motor_states = []        # 电机运行状态 0/1
        self.sensor_data = []         # 当前传感器数据 [pitch, roll, yaw]



        # ========== 臂体弯曲角度（两段） ==========
        self.target_angle1 = 0.0       # 第一段目标角度
        self.current_angle1 = 0.0      # 第一段当前角度
        self.target_angle2 = 0.0
        self.current_angle2 = 0.0
        self.dir1 = 0                  # 第一段方向 (0上 1右 2下 3左)
        self.dir2 = 0
        self.angle1_modified = False   # 用户是否手动修改了输入框
        self.angle2_modified = False

        # 滤波相关
        self.angle_filter_alpha = 0.3
        self.filtered_bend_angle = 0.0   # 用于闭环的第二段滤波角度
        self.current_bend_angle = 0.0    # 当前弯曲角度（通常为第二段）
        self.current_area_change = 0.0   # 面积变化率（可选）

        # ========== 历史数据缓存（曲线） ==========
        self.hist_time = []            # 时间轴（秒）
        self.hist_motors = []          # 电机历史数据
        self.hist_sensors = []         # 传感器历史数据
        # 弯曲角度专用历史（四条曲线）
        self.hist_bend_time = []
        self.hist_target1 = []
        self.hist_current1 = []
        self.hist_target2 = []
        self.hist_current2 = []

        # ========== UI 组件引用 ==========
        self.m_page = 0
        self.s_page = 0
        self.cards_motor = []
        self.cards_sensor = []
        self.bend_graph_window = None
        self.bend_graph_controller = None
        self.active_graph = None            # 旧版，保留兼容
        self.active_type = None             # 'motor' 或 'sensor'
        self.active_graph_ui = None
        self.active_graph_controller = None

        # ========== PID 闭环控制 ==========
        self.pid = PID(
            Kp=1, Ki=0.01, Kd=0.01, dt=0.2,
            output_limits=(-70, 70), integral_limits=(-20, 20)
        )
        self.last_sent_angle = None
        self.angle_deadband = 1

        # ========== 定时器 ==========
        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self.closed_loop_control)
        self.control_timer.start(200)

        self.history_timer = QTimer()
        self.history_timer.timeout.connect(self.record_history)
        self.history_timer.start(10)

        # ========== 数据处理（滤波） ==========
        self.data_filter = DataFilter(window_size=3)

        # ========== 初始化界面并启动工作线程 ==========
        self.init_ui()
        self.worker.start()

    def init_ui(self):
        main_layout = QHBoxLayout(self)
        splitter = QSplitter(Qt.Horizontal)

        content_widget = QWidget()
        main_layout = QHBoxLayout(content_widget)
        splitter = QSplitter(Qt.Horizontal)

        # 左侧面板
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_widget.setMaximumWidth(1000)
        g_power = QGroupBox("1. 系统操作权限")
        l_power = QHBoxLayout(g_power)

        self.btn_toggle = AnimatedButton("▶ 启动控制系统","#107C10", "#063A06")
        self.btn_toggle.setCheckable(True)

        shadow = QGraphicsDropShadowEffect()
        self.btn_toggle.toggled.connect(self.sys_toggle)
        self.btn_toggle.setGraphicsEffect(shadow)

        self.btn_stop = AnimatedButton("⏹紧急停止","red", "#A80000")
        shadow = QGraphicsDropShadowEffect()
        self.btn_stop.setGraphicsEffect(shadow)
        self.btn_stop.setProperty("class", "emergency")
        self.btn_stop.clicked.connect(self.sys_stop)

        l_power.addWidget(self.btn_toggle)
        l_power.addWidget(self.btn_stop)
        left_layout.addWidget(g_power)

        g_quick = QGroupBox("2. 臂体弯曲控制")
        l_quick = QVBoxLayout(g_quick)
        self.btn_home = AnimatedButton("⌂ 一键归中","#1E1E1E","#505050")
        self.btn_home.clicked.connect(self.send_home_command)
        l_quick.addWidget(self.btn_home)

        l_section1 = QHBoxLayout()
        self.spin_bend1 = self._create_custom_spinbox(0, 90, 0, prefix="第一段角度: ", suffix='°')
        self.spin_bend1.spin.valueChanged.connect(lambda: setattr(self, 'angle1_modified', True))

        self.btn_s1_up = AnimatedButton("↑上", "#1E1E1E", "#505050")
        self.btn_s1_down = AnimatedButton("↓下", "#1E1E1E", "#505050")
        self.btn_s1_left = AnimatedButton("←左", "#1E1E1E", "#505050")
        self.btn_s1_right = AnimatedButton("→右", "#1E1E1E", "#505050")
        self.btn_s1_up.clicked.connect(lambda: self.direction_bend('1', "up"))
        self.btn_s1_down.clicked.connect(lambda: self.direction_bend('1', "down"))
        self.btn_s1_left.clicked.connect(lambda: self.direction_bend('1', "left"))
        self.btn_s1_right.clicked.connect(lambda: self.direction_bend('1', "right"))

        l_section1.addWidget(self.spin_bend1)
        l_section1.addWidget(self.btn_s1_up)
        l_section1.addWidget(self.btn_s1_down)
        l_section1.addWidget(self.btn_s1_left)
        l_section1.addWidget(self.btn_s1_right)

        self.btn_bend_all = AnimatedButton("臂体弯曲", "#00BCD4", "#505050")
        self.btn_bend_all.clicked.connect(self.send_bend_combined)
        l_section1.addWidget(self.btn_bend_all)        # 臂体弯曲

        l_quick.addLayout(l_section1)

        l_section2 = QHBoxLayout()
        self.spin_bend2 = self._create_custom_spinbox(0, 70, 0, prefix="第二段角度: ", suffix="°")
        self.spin_bend2.spin.valueChanged.connect(lambda: setattr(self, 'angle2_modified', True))

        self.btn_s2_up = AnimatedButton("↑上", "#1E1E1E", "#505050")
        self.btn_s2_down = AnimatedButton("↓下", "#1E1E1E", "#505050")
        self.btn_s2_left = AnimatedButton("←左", "#1E1E1E", "#505050")
        self.btn_s2_right = AnimatedButton("→右", "#1E1E1E", "#505050")
        self.btn_s2_up.clicked.connect(lambda: self.direction_bend('2', "up"))
        self.btn_s2_down.clicked.connect(lambda: self.direction_bend('2', "down"))
        self.btn_s2_left.clicked.connect(lambda: self.direction_bend('2', "left"))
        self.btn_s2_right.clicked.connect(lambda: self.direction_bend('2', "right"))
        # 臂体弯曲按钮（发送组合指令）



        self.btn_closed_bend = AnimatedButton("循环寿命检测","#FF8C00","#B85C00")
        self.btn_closed_bend.clicked.connect(self.send_closed_loop_bend_command)

        l_section2.addWidget(self.spin_bend2)
        l_section2.addWidget(self.btn_s2_up)
        l_section2.addWidget(self.btn_s2_down)
        l_section2.addWidget(self.btn_s2_left)
        l_section2.addWidget(self.btn_s2_right)
        l_quick.addLayout(l_section2)


        l_section2.addWidget(self.btn_closed_bend)     # 闭环弯曲

        h_pid = QHBoxLayout()
        self.spin_kp = self._create_custom_spinbox(0, 10, 0.5, prefix="kp: ", step=0.1)
        h_pid.addWidget(self.spin_kp)
        self.spin_ki = self._create_custom_spinbox(0, 10, 0, prefix="ki: ", step=0.01)
        h_pid.addWidget(self.spin_ki)
        self.spin_kd = self._create_custom_spinbox(0, 10, 0, prefix="kd: ", step=0.01)
        h_pid.addWidget(self.spin_kd)
        btn_apply_pid = AnimatedButton("应用PID参数", "#1E1E1E","#505050")
        btn_apply_pid.clicked.connect(self.apply_pid_params)
        h_pid.addWidget(btn_apply_pid)
        l_quick.addLayout(h_pid)

        left_layout.addWidget(g_quick)

        g_addr = QGroupBox("3. 电机控制")
        f_addr = QGridLayout(g_addr)
        self.cb_motor_id = QComboBox()
        self.spin_m_pos = self._create_custom_spinbox(-80, 80, 0, prefix="位移：", suffix='mm')
        self.spin_m_vel = self._create_custom_spinbox(-20, 20, 10, prefix="速度: ", suffix=" mm/s")
        self.spin_m_acc = self._create_custom_spinbox(-10, 10, 10, prefix="加速度: ", suffix=" mm/s^2")
        self.btn_send_m = AnimatedButton("发至电机","#00BCD4","#505050")
        self.btn_send_m.clicked.connect(self.send_motor)
        f_addr.addWidget(QLabel("电机ID:"), 0, 0)
        f_addr.addWidget(self.cb_motor_id, 0, 1)
        f_addr.addWidget(self.btn_send_m, 0, 2)
        self.motor_status_ball = QLabel("●")
        self.motor_status_ball.setStyleSheet("color: #c46b5b; font-size: 8pt;")
        f_addr.addWidget(self.motor_status_ball, 0, 3)
        self.cb_motor_id.currentIndexChanged.connect(self.update_motor_status_ball)
        f_addr.addWidget(self.spin_m_pos, 1, 0)
        f_addr.addWidget(self.spin_m_vel, 1, 1)
        f_addr.addWidget(self.spin_m_acc, 1, 2)

        # ========== 循环计数显示（新增） ==========
        h_cycle = QHBoxLayout()
        self.lbl_cycle_count = QLabel("循环次数: 0")
        self.lbl_cycle_count.setStyleSheet("font-weight: bold; color: #b5956b;")
        btn_reset_cycle = AnimatedButton("重置计数", "#1E1E1E", "#505050")
        btn_reset_cycle.clicked.connect(self.reset_cycle_count)
        h_cycle.addWidget(self.lbl_cycle_count)
        h_cycle.addStretch()
        h_cycle.addWidget(btn_reset_cycle)
        # 放置于第2行，占3列
        f_addr.addLayout(h_cycle, 2, 0, 1, 3)

        left_layout.addWidget(g_addr)

        g_sensor = QGroupBox("4. 压力数据监控")
        l_sensor = QVBoxLayout(g_sensor)

        # 传感器选择行
        h_sensor_line = QHBoxLayout()
        h_sensor_line.addWidget(QLabel("压力传感器 ID:"))
        self.cb_sensor_monitor = QComboBox()
        self.cb_sensor_monitor.currentIndexChanged.connect(self.update_sensor_monitor)
        h_sensor_line.addWidget(self.cb_sensor_monitor)
        l_sensor.addLayout(h_sensor_line)

        # 新增：校准值输入与校准按钮行
        h_calib = QHBoxLayout()
        h_calib.addWidget(QLabel("校准值:"))
        self.sensor_calib_spin = QDoubleSpinBox()
        self.sensor_calib_spin.setRange(-999.99, 999.99)
        self.sensor_calib_spin.setDecimals(2)
        self.sensor_calib_spin.setValue(0.00)
        self.sensor_calib_spin.setSuffix(" N")
        self.sensor_calib_spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.sensor_calib_spin.setStyleSheet("""
            QDoubleSpinBox {
                min-height: 34px;
                font-size: 10pt;
                font-weight: bold;
                border: 2px solid #b0b0b0;
                border-radius: 10px;
                background: white;
                padding-right: 5px;
            }
            QDoubleSpinBox:focus { border-color: #0078D7; }
        """)
        h_calib.addWidget(self.sensor_calib_spin)

        self.btn_calib = AnimatedButton("校准传感器", "#1E1E1E", "#505050")
        self.btn_calib.clicked.connect(self.calibrate_sensor)
        h_calib.addWidget(self.btn_calib)
        l_sensor.addLayout(h_calib)

        left_layout.addWidget(g_sensor)

        left_layout.addStretch()

        # 右侧看板
        right_widget = QWidget()
        right_widget.setMaximumWidth(1000)
        self.right_layout = QVBoxLayout(right_widget)
        self.tabs = QTabWidget()
        tab_all = QWidget()
        v_all = QVBoxLayout(tab_all)
        self.grid_m = QGridLayout()
        h_m_page = QHBoxLayout()
        self.btn_m_prev = AnimatedButton("◀ 上一页", "grey","#505050")
        self.btn_m_prev.clicked.connect(lambda: self.change_page('m', -1))
        self.btn_m_prev.setProperty("class", "page-btn")
        self.btn_m_next = AnimatedButton("下一页 ▶", "grey","#505050")
        self.btn_m_next.clicked.connect(lambda: self.change_page('m', 1))
        self.btn_m_next.setProperty("class", "page-btn")
        self.lbl_m_page = QLabel("电机 1/1 页")
        self.lbl_m_page.setProperty("class", "page-btn")
        self.lbl_m_page.setAlignment(Qt.AlignCenter)
        h_m_page.addWidget(self.btn_m_prev)
        h_m_page.addWidget(self.lbl_m_page)
        h_m_page.addWidget(self.btn_m_next)
        self.grid_s = QGridLayout()
        h_s_page = QHBoxLayout()
        self.btn_s_prev = AnimatedButton("◀ 上一页", "grey","#505050")
        self.btn_s_prev.clicked.connect(lambda: self.change_page('s', -1))
        self.btn_s_prev.setProperty("class", "page-btn")
        self.btn_s_next = AnimatedButton("下一页 ▶", "grey","#505050")
        self.btn_s_next.clicked.connect(lambda: self.change_page('s', 1))
        self.btn_s_next.setProperty("class", "page-btn")
        self.lbl_s_page = QLabel("压力传感器 1/1 页")
        self.lbl_s_page.setProperty("class", "page-btn")
        self.lbl_s_page.setAlignment(Qt.AlignCenter)
        h_s_page.addWidget(self.btn_s_prev)
        h_s_page.addWidget(self.lbl_s_page)
        h_s_page.addWidget(self.btn_s_next)
        v_all.addLayout(h_m_page)
        v_all.addLayout(self.grid_m)
        v_all.addStretch()
        v_all.addLayout(h_s_page)
        v_all.addLayout(self.grid_s)
        v_all.addStretch()
        self.tabs.addTab(tab_all, "👁 电机与压力传感器数据监控")

        tab_bend = QWidget()
        v_bend = QVBoxLayout(tab_bend)

        # 第一段：目标角度 + 当前角度
        hbox_angle1 = QHBoxLayout()
        self.target_angle1_card, self.target_angle1_val = self.create_flat_card(
            "第一段目标角度 (deg)", "0.00", "#b5956b"
        )
        hbox_angle1.addWidget(self.target_angle1_card)

        self.current_angle1_card, self.current_angle1_val = self.create_flat_card(
            "第一段当前角度 (deg)", "0.00", "#b5956b"
        )
        hbox_angle1.addWidget(self.current_angle1_card)
        v_bend.addLayout(hbox_angle1)

        # 第二段：目标角度 + 当前角度
        hbox_angle2 = QHBoxLayout()
        self.target_angle2_card, self.target_angle2_val = self.create_flat_card(
            "第二段目标角度 (deg)", "0.00", "#b5956b"
        )
        hbox_angle2.addWidget(self.target_angle2_card)

        self.current_angle2_card, self.current_angle2_val = self.create_flat_card(
            "第二段当前角度 (deg)", "0.00", "#b5956b"
        )
        hbox_angle2.addWidget(self.current_angle2_card)
        v_bend.addLayout(hbox_angle2)

        self.tabs.addTab(tab_bend, "🔧 柔性臂运动数据监控")

        tab_single = QWidget()
        v_single = QVBoxLayout(tab_single)
        h_sel = QHBoxLayout()
        self.cb_view_type = QComboBox()
        self.cb_view_type.addItems(["定点监测: 电机", "定点监测: 压力传感器"])
        self.cb_view_type.currentIndexChanged.connect(self.update_single_monitor_labels)
        self.cb_view_id = QComboBox()
        self.cb_view_id.currentIndexChanged.connect(lambda: self.update_ui())
        h_sel.addWidget(self.cb_view_type)
        h_sel.addWidget(self.cb_view_id)
        h_sel.addStretch()
        v_single.addLayout(h_sel)
        self.single_cards = []
        # 初始标题和颜色，后续会根据类型动态变化
        init_titles = [("位移 (mm)", "#b5956b"), ("速度 (mm/s)", "#b5956b"), ("加速度 (mm/s²)", "#b5956b")]
        for default_title, default_color in init_titles:
            card_frame = QFrame()
            card_frame.setStyleSheet(
                "QFrame { background: #fdfaf5; border: 1px solid #c4b49a; border-radius: 6px; }"
            )
            card_layout = QHBoxLayout(card_frame)
            title_label = QLabel(default_title)
            title_label.setStyleSheet("color: #5a4636; font-weight:bold; border:none; font-size:15pt;")
            value_label = QLabel("0.00")
            value_label.setStyleSheet(
                f"color: {default_color}; font-size: 15pt; font-weight: bold; border: none;"
            )
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            card_layout.addWidget(title_label)
            card_layout.addStretch()
            card_layout.addWidget(value_label)
            v_single.addWidget(card_frame)
            self.single_cards.append((title_label, value_label))
        self.tabs.addTab(tab_single, "🎯 定点监测(电机与压力传感器)")

        self.right_layout.addWidget(self.tabs)
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 3)
        main_layout.addWidget(splitter)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(content_widget)
        scroll_area.setMaximumHeight(900)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        # 统一的新样式滚动条
        scroll_area.setStyleSheet("""
            QScrollBar:vertical {
                background: #fbf7f0;
                width: 10px;
                margin: 0;
                border-radius: 5px;
            }
            QScrollBar::handle:vertical {
                background: #c4b49a;
                min-height: 20px;
                border-radius: 5px;
            }
            QScrollBar::handle:vertical:hover {
                background: #b5956b;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
            QScrollBar:horizontal {
                background: #fbf7f0;
                height: 10px;
                border-radius: 5px;
            }
            QScrollBar::handle:horizontal {
                background: #c4b49a;
                min-width: 20px;
                border-radius: 5px;
            }
            QScrollBar::handle:horizontal:hover {
                background: #b5956b;
            }
            QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
                width: 0px;
            }
        """)
        self.setLayout(QVBoxLayout())
        self.layout().addWidget(scroll_area)

        self.rebuild_cards()
    def update_single_monitor_labels(self):
        is_motor = (self.cb_view_type.currentIndex() == 0)
        if is_motor:
            titles = ["位移 (mm)", "速度 (mm/s)", "加速度 (mm/s²)"]
            colors = ["#b5956b", "#b5956b", "#b5956b"]
            self.update_single_monitor_ids(range(1, self.num_m + 1))
        else:
            titles = ["raw_val (N)", "filter_val (N)", "val (N)"]
            colors = ["#b5956b", "#b5956b", "#b5956b"]
            self.update_single_monitor_ids(range(1, self.num_s + 1))
        for i, (title_label, value_label) in enumerate(self.single_cards):
            title_label.setText(titles[i])
            value_label.setStyleSheet(f"color: {colors[i]}; font-size: 15pt; font-weight: bold; border: none;")
        self.update_ui()

    def update_single_monitor_ids(self, ids_range):
        """更新定点监测的ID下拉框选项，ids_range是一个可迭代的ID列表（如range(1, num+1)）"""
        current_id = self.cb_view_id.currentText()
        self.cb_view_id.blockSignals(True)
        self.cb_view_id.clear()
        id_list = [f"ID {i}" for i in ids_range]
        if id_list:
            self.cb_view_id.addItems(id_list)
            # 尝试恢复之前选中的ID
            if current_id in id_list:
                self.cb_view_id.setCurrentText(current_id)
            else:
                self.cb_view_id.setCurrentIndex(0)
        else:
            self.cb_view_id.addItem("无")
        self.cb_view_id.blockSignals(False)

    def create_motor_card(self, title, color):
        frame = QFrame()
        frame.setObjectName("motorCard")
        frame.setStyleSheet("""
            QFrame#motorCard { 
                background: #fdfaf5; 
                border: 1px solid #c4b49a; 
                border-radius: 6px;
            }
            QFrame#motorCard:hover {
                border: 1px solid #b5956b;
            }
        """)
        main_layout = QVBoxLayout(frame)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(10)
        top_widget = QWidget()
        top_layout = QHBoxLayout(top_widget)
        top_layout.setContentsMargins(0, 0, 0, 0)
        title_label = QLabel(title)
        title_label.setStyleSheet("color: #5a4636; font-weight: bold; font-size: 10pt; border: none;")
        title_label.setAlignment(Qt.AlignCenter)
        top_layout.addWidget(title_label)
        top_layout.addStretch()
        state_ball = QLabel("●")
        state_ball.setStyleSheet("color: #888; font-size: 8pt; border: none;")
        top_layout.addWidget(state_ball)
        main_layout.addWidget(top_widget)

        def create_block(block_name, unit, color):
            block_widget = QWidget()
            block_layout = QVBoxLayout(block_widget)
            block_layout.setContentsMargins(0, 0, 0, 0)
            block_layout.setSpacing(4)
            title_lbl = QLabel(f"{block_name} ({unit})")
            title_lbl.setStyleSheet(f"color: {color}; font-size: 10pt; font-weight: bold; border: none;")
            title_lbl.setAlignment(Qt.AlignCenter)
            block_layout.addWidget(title_lbl)
            value_widget = QWidget()
            value_layout = QHBoxLayout(value_widget)
            value_layout.setContentsMargins(0, 0, 0, 0)
            value_layout.setSpacing(30)
            cur_label = QLabel("当前: 0.00")
            cur_label.setStyleSheet("color: #2e241b; font-size: 8pt; font-weight: bold; border: none;")
            cur_label.setAlignment(Qt.AlignCenter)
            tar_label = QLabel("目标: 0.00")
            tar_label.setStyleSheet("color: #b5956b; font-size: 8pt; border: none;")
            tar_label.setAlignment(Qt.AlignCenter)
            value_layout.addStretch()
            value_layout.addWidget(cur_label)
            value_layout.addWidget(tar_label)
            value_layout.addStretch()
            block_layout.addWidget(value_widget)
            return block_widget, cur_label, tar_label

        # 位移、速度、加速度标题使用统一的金棕色
        block_pos, cur_pos, tar_pos = create_block("位移", "mm", "#b5956b")
        block_vel, cur_vel, tar_vel = create_block("速度", "mm/s", "#b5956b")
        block_acc, cur_acc, tar_acc = create_block("加速度", "mm/s²", "#b5956b")
        main_layout.addWidget(block_pos)
        line1 = QFrame()
        line1.setFrameShape(QFrame.HLine)
        line1.setStyleSheet("background-color: #d9cfbd; border: none; height: 1px;")
        main_layout.addWidget(line1)
        main_layout.addWidget(block_vel)
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setStyleSheet("background-color: #d9cfbd; border: none; height: 1px;")
        main_layout.addWidget(line2)
        main_layout.addWidget(block_acc)
        lbls = [cur_pos, tar_pos, cur_vel, tar_vel, cur_acc, tar_acc, state_ball]
        return frame, lbls

    def create_sensor_card(self, title, color):
        frame = QFrame()
        frame.setObjectName("sensorCard")
        frame.setStyleSheet("""
            QFrame#sensorCard { 
                background: #fdfaf5; 
                border: 1px solid #c4b49a; 
                border-radius: 6px;
            }
            QFrame#sensorCard:hover {
                border: 1px solid #b5956b;
            }
        """)
        main_layout = QVBoxLayout(frame)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(8)
        top_widget = QWidget()
        top_layout = QHBoxLayout(top_widget)
        top_layout.setContentsMargins(0, 0, 0, 0)
        title_label = QLabel(title)
        title_label.setStyleSheet("color: #5a4636; font-weight: bold; font-size: 10pt; border: none;")
        title_label.setAlignment(Qt.AlignLeft)
        top_layout.addWidget(title_label)
        main_layout.addWidget(top_widget)

        def create_axis_block(axis_name, unit):
            block_widget = QWidget()
            block_layout = QHBoxLayout(block_widget)
            block_layout.setContentsMargins(0, 0, 0, 0)
            block_layout.setSpacing(10)
            block_layout.addStretch()
            label = QLabel(f"{axis_name} ({unit}):")
            label.setStyleSheet(f"color: {color}; font-size: 8pt; font-weight: bold; border: none;")
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            block_layout.addWidget(label)
            value_label = QLabel("0.00")
            value_label.setStyleSheet(f"color: #2e241b; font-size: 8pt; font-weight: bold; border: none;")
            value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            block_layout.addWidget(value_label)
            block_layout.addStretch()
            return block_widget, value_label

        block_pitch, val_pitch = create_axis_block("raw_val", "N")
        block_roll, val_roll = create_axis_block("filter_val" , "N")
        block_yaw, val_yaw = create_axis_block("val", "N")
        main_layout.addWidget(block_pitch)
        line1 = QFrame()
        line1.setFrameShape(QFrame.HLine)
        line1.setStyleSheet("background-color: #d9cfbd; border: none; height: 1px;")
        main_layout.addWidget(line1)
        main_layout.addWidget(block_roll)
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setStyleSheet("background-color: #d9cfbd; border: none; height: 1px;")
        main_layout.addWidget(line2)
        main_layout.addWidget(block_yaw)
        lbls = [val_pitch, val_roll, val_yaw]
        return frame, lbls

    def create_flat_card(self, title, val, color):
        frame = QFrame()
        frame.setStyleSheet(
            "QFrame { background: #fdfaf5; border: 1px solid #c4b49a; border-radius: 6px; }"
        )
        layout = QHBoxLayout(frame)
        lbl_val = QLabel(val)
        lbl_val.setStyleSheet(f"color: {color}; font-size: 12pt; font-weight: bold; border: none;")
        layout.addWidget(QLabel(title, styleSheet="color: #5a4636; font-weight:bold; border:none; font-size:12pt;"))
        layout.addStretch()
        layout.addWidget(lbl_val)
        return frame, lbl_val

    def expand_device(self, dev_type):
        if dev_type == 'motor':
            self.num_m += 1
            self.motor_data.append([0.0, 0.0, 0.0])
            self.motor_states.append(0)
            self.motor_target.append([0.0, 0.0, 0.0])
            self.hist_motors = [step + [[0.0, 0.0, 0.0]] for step in self.hist_motors]
            self.m_page = 0
        else:
            self.num_s += 1
            self.sensor_data.append([10.0, 0.0, 0.0])  # 虚拟传感器 pitch=10.0
            self.hist_sensors = [step + [[10.0, 0.0, 0.0]] for step in self.hist_sensors]
            self.s_page = 0
        self.rebuild_cards()
        self.refresh_pagination()
        self.logger(f"🔧 成功扩容了一个{dev_type}，当前 M:{self.num_m}, S:{self.num_s}", port=self.port_name)

    def rebuild_cards(self):
        for i in reversed(range(self.grid_m.count())):
            self.grid_m.itemAt(i).widget().setParent(None)
        for i in reversed(range(self.grid_s.count())):
            self.grid_s.itemAt(i).widget().setParent(None)
        self.cb_motor_id.clear()
        self.cb_sensor_monitor.clear()
        self.cb_view_id.clear()
        if self.num_m > 0:
            while len(self.motor_data) < self.num_m:
                self.motor_data.append([0.0, 0.0, 0.0])
                self.motor_target.append([0.0, 0.0, 0.0])
            while len(self.motor_states) < self.num_m:
                self.motor_states.append(0)
            self.motor_data = self.motor_data[:self.num_m]
            self.motor_states = self.motor_states[:self.num_m]
        else:
            self.motor_data = []
            self.motor_states = []
        if self.num_s > 0:
            while len(self.sensor_data) < self.num_s:
                self.sensor_data.append([0.0, 0.0, 0.0])
            self.sensor_data = self.sensor_data[:self.num_s]
        else:
            self.sensor_data = []
        self.cards_motor = []
        for i in range(self.num_m):
            card, lbls = self.create_motor_card(f"电机 ID:{i + 1}", "#000")
            self.cards_motor.append((card, lbls))
            self.grid_m.addWidget(card, 0, i % 3)
            self.cb_motor_id.addItem(f"电机 {i + 1}")
        self.cards_sensor = []
        for i in range(self.num_s):
            card, lbls = self.create_sensor_card(f"压力传感器 ID:{i + 1}", "#D83B01")
            self.cards_sensor.append((card, lbls))
            self.grid_s.addWidget(card, 0, i % 3)
            self.cb_sensor_monitor.addItem(f"压力传感器 {i + 1}")
        max_id = max(self.num_m, self.num_s)
        self.cb_view_id.addItems([f"ID {i + 1}" for i in range(max_id)])
        self.refresh_pagination()
        self.update_single_monitor_labels() # 确保ID列表与当前数量同步
        self.update_ui()

    def change_page(self, t, delta):
        if t == 'm':
            self.m_page += delta
        else:
            self.s_page += delta
        self.refresh_pagination()

    def refresh_pagination(self):
        m_pages = max(1, math.ceil(self.num_m / 3))
        self.m_page = max(0, min(self.m_page, m_pages - 1))
        self.lbl_m_page.setText(f"电机 {self.m_page + 1}/{m_pages} 页")
        for i, (card, _) in enumerate(self.cards_motor):
            card.setVisible(self.m_page * 3 <= i < (self.m_page + 1) * 3)
        s_pages = max(1, math.ceil(self.num_s / 3))
        self.s_page = max(0, min(self.s_page, s_pages - 1))
        self.lbl_s_page.setText(f"压力传感器 {self.s_page + 1}/{s_pages} 页")
        for i, (card, _) in enumerate(self.cards_sensor):
            card.setVisible(self.s_page * 3 <= i < (self.s_page + 1) * 3)

    # ------------------ 系统控制 ------------------

    def sys_toggle(self,checked):
        """开关按钮状态变化时的处理函数"""
        if checked:
            # 按钮被按下（开启状态）
            self.btn_toggle.setText("⏹ 关闭控制系统")
            self.btn_toggle.set_normal_color("#D13438")  # 改为危险样式（红色）
            self.btn_toggle.set_hover_color("#6B1418")
            # 刷新样式表，使属性生效
            self.btn_toggle.style().unpolish(self.btn_toggle)
            self.btn_toggle.style().polish(self.btn_toggle)
            self.sys_start()   # 启动系统
        else:
            # 按钮弹起（关闭状态）
            self.btn_toggle.setText("▶ 启动控制系统")
            self.btn_toggle.set_normal_color("#107C10")  # 恢复成功样式（绿色）
            self.btn_toggle.set_hover_color("#063A06")
            self.btn_toggle.style().unpolish(self.btn_toggle)
            self.btn_toggle.style().polish(self.btn_toggle)
            self.sys_close()    # 关闭系统

    def sys_close(self):
        self.is_started = False
        self.closed_loop_enabled = False   # 停止闭环
        self.send_cmd(0x00, "失能", "关闭控制系统", is_motor=True)

    def sys_start(self):
        if self.is_started:
            return
        self.is_started = True

        self.send_cmd(0x01, "使能", "启动控制系统", is_motor=True)

    def sys_stop(self):
        if self.is_started:
            # 临时阻止信号，避免 setChecked 触发 toggled 导致递归
            self.btn_toggle.blockSignals(True)

            # 保持 checkable=True，只改变 checked 状态
            self.btn_toggle.setChecked(False)   # ✅ 不是 setCheckable(False)
            self.btn_toggle.setText("▶ 启动控制系统")
            self.btn_toggle.set_normal_color("#107C10")
            self.btn_toggle.set_hover_color("#063A06")
            self.btn_toggle.style().unpolish(self.btn_toggle)
            self.btn_toggle.style().polish(self.btn_toggle)
            # 恢复信号（尽快恢复，避免长时间阻塞）
            self.btn_toggle.blockSignals(False)

        self.is_started = False
        self.closed_loop_enabled = False   # 停止闭环
        if hasattr(self, 'btn_closed_bend') and self.btn_closed_bend.text() == "⏹ 停止循环弯曲":
            self.spin_bend2.spin.setEnabled(True)
            self.btn_s2_up.setEnabled(True)
            self.btn_s2_down.setEnabled(True)
            self.btn_s2_left.setEnabled(True)
            self.btn_s2_right.setEnabled(True)
            self.btn_bend_all.setEnabled(True)
            self.btn_closed_bend.setText("循环弯曲")
            self.btn_closed_bend.set_normal_color("#FF8C00")
        # 发送紧急停止命令
        self.send_cmd(0x02, "紧急停止", "LQTS紧急停止按钮", is_motor=True)

    def handle_serial_error(self, error_msg):
        self.serial_error = True
        if self.history_timer.isActive():
            self.history_timer.stop()
        self.hist_time.clear()
        self.hist_motors.clear()
        self.hist_sensors.clear()
        self.logger(f"❌ 串口异常: {error_msg}", port=self.port_name)
        msg_box = QMessageBox(self)
        msg_box.setIcon(QMessageBox.Critical)
        msg_box.setWindowTitle("串口断连")
        msg_box.setText(f"当前串口设备 {self.port_name} 已断开连接！")
        msg_box.setInformativeText("请关闭当前数据页面，重新连接串口设备。")
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()
        # 禁用所有操作按钮
        buttons = [
            self.btn_toggle, self.btn_stop, self.btn_home, self.btn_closed_bend,self.btn_bend_all,
            self.btn_s1_up, self.btn_s1_down, self.btn_s1_left, self.btn_s1_right,
            self.btn_s2_up, self.btn_s2_down, self.btn_s2_left, self.btn_s2_right,
            self.btn_send_m, self.btn_m_prev, self.btn_m_next,
            self.btn_s_prev, self.btn_s_next,
        ]
        for btn in buttons:
            if btn:                     # 防止 None
                btn.setEnabled(False)

    def send_cmd(self, func_code, action, detail, data=b'', is_motor=True):

        debug_mode = False
        if self.debug_check:
            debug_mode = self.debug_check()
        try:
            if func_code not in [0x00, 0x01, 0x02, 0x04,0X05, 0x06, 0xFE] and not self.is_started and not debug_mode:
                error_msg = "请先点击启动控制系统"
                QMessageBox.warning(self, "拒绝", error_msg)
                self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
                return

            frame_head = 0xAA if is_motor else 0xBB
            frame = struct.pack('>BBB', frame_head, func_code, len(data)) + data
            frame += bytes([sum(frame) & 0xFF])

            # Debug 模式下：如果串口实际未连接，可以选择不真实发送，仅记录日志
            if debug_mode and self.serial_error:
                # 只记录，不实际发送（可选）
                self.logger(f"📤 [DEBUG] {action} -> {detail} (模拟发送)", raw_data=frame, level="DEBUG", port=self.port_name)
                return

            self.worker.send_data(frame)
            GlobalHistory.add_record(self.port_name, action, detail, frame.hex().upper())

            if func_code == 0x02:
                self.logger(f"📤 {action} -> {detail}", raw_data=frame, level="WARNING", port=self.port_name)
            else:
                self.logger(f"📤 {action} -> {detail}", raw_data=frame, port=self.port_name)


        except serial.SerialException as e:
            error_msg = f"串口通信失败: {str(e)}"
            QMessageBox.critical(self, "串口错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
        except Exception as e:
            error_msg = f"发送命令失败: {str(e)}"
            QMessageBox.critical(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)

    def send_motor(self):
        debug_mode = self.debug_check() if self.debug_check else False

        # 检查是否无电机
        if self.num_m == 0 and not debug_mode:
            error_msg = "当前没有可用的电机设备，无法进行电机控制"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        # Debug 模式下即使没有电机，也使用默认ID=1，假数据发送
        if debug_mode and self.num_m == 0:
            m_id = 1
            pos, vel, acc = 0, 10, 10
        else:
            m_id = self.cb_motor_id.currentIndex() + 1
            if m_id > self.num_m:
                error_msg = f"电机ID {m_id} 无效，当前只有 {self.num_m} 个电机"
                QMessageBox.warning(self, "错误", error_msg)
                self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
                return
            pos = int(self.spin_m_pos.spin.value() * 100)
            vel = int(self.spin_m_vel.spin.value() * 100)
            acc = int(self.spin_m_acc.spin.value() * 100)

        # 更新目标值（即使在 debug 模式下也可更新，视需要）
        while len(self.motor_target) < self.num_m:
            self.motor_target.append([0.0, 0.0, 0.0])
        if m_id <= len(self.motor_target):
            self.motor_target[m_id-1] = [pos/100, vel/100, acc/100]

        direction = 0 if pos >= 0 else 1
        distance = abs(pos)
        data = struct.pack('>BBHHH', m_id, direction, distance, vel, acc)
        self.send_cmd(0x03, f"控制电机{m_id}", f"位移:{pos/100}, 速度:{vel/100}, 加速度:{acc/100}", data, is_motor=True)

    def calibrate_sensor(self):
        if self.num_s == 0:
            error_msg = "当前没有可用的压力传感器，无法进行传感器校准"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        idx = self.cb_sensor_monitor.currentIndex() + 1
        if idx > self.num_s:
            error_msg = f"压力传感器 ID {idx} 无效，当前只有 {self.num_s} 个传感器"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        try:
            self.send_cmd(0x03, f"校准压力传感器{idx}", f"Sensor {idx} 校准", struct.pack('>B', idx), is_motor=False)
        except Exception as e:
            error_msg = f"发送压力传感器校准命令失败: {str(e)}"
            QMessageBox.critical(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)

    def read_sensor_data(self):
        if self.num_s == 0:
            error_msg = "当前没有可用的压力传感器，无法读取传感器"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        idx = self.cb_sensor_monitor.currentIndex() + 1
        if idx > self.num_s:
            error_msg = f"压力传感器 ID {idx} 无效，当前只有 {self.num_s} 个传感器"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        try:
            self.send_cmd(0x01, f"读取压力传感器{idx}", f"请求压力{idx}数据", struct.pack('>B', idx), is_motor=False)
        except Exception as e:
            error_msg = f"发送读取压力传感器数据命令失败: {str(e)}"
            QMessageBox.critical(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)

    # 电机与臂体控制

    def send_home_command(self):
        debug_mode = self.debug_check() if self.debug_check else False
        if not self.is_started and not debug_mode:
            error_msg = "请先点击启动控制系统"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return
        if self.num_m == 0:
            error_msg = "当前没有可用的电机设备，无法归中"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        try:
            count = self.num_m
            start_addr = 1
            distances = [0] * count
            data = struct.pack('>BB', count, start_addr)
            for dist in distances:
                data += struct.pack('>H', dist)
            self.send_cmd(0x04, "一键归中", "所有电机距离复位为0", data, is_motor=True)

            # 两段角度目标归零
            self.target_angle1 = 0
            self.target_angle2 = 0
            self.angle1_modified = False
            self.angle2_modified = False
            if hasattr(self, 'spin_bend1'):
                self.spin_bend1.spin.setValue(0)
            if hasattr(self, 'spin_bend2'):
                self.spin_bend2.spin.setValue(0)
        except Exception as e:
            error_msg = f"发送一键归中命令失败: {str(e)}"
            QMessageBox.critical(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)

    # ========== 臂体控制 ==========

    def send_bend_combined(self, angle1=None, angle2=None, dir1=None, dir2=None, log_enabled=True):
        debug_mode = self.debug_check() if self.debug_check else False
        if not self.is_started and not debug_mode:
            if log_enabled:
                QMessageBox.warning(self, "错误", "请先点击启动控制系统")
            return
        if self.num_m == 0 and not debug_mode:
            if log_enabled:
                QMessageBox.warning(self, "错误", "当前没有可用的电机设备")
            return

        # 获取角度
        a1 = self.spin_bend1.spin.value()
        a2 = self.spin_bend2.spin.value()

        # 绝对值限幅
        a1 = min(abs(float(a1)), 90.0)
        a2 = min(abs(float(a2)), 70.0)

        # 方向索引
        d1 = dir1 if dir1 is not None else self.dir1
        d2 = dir2 if dir2 is not None else self.dir2

        angle1_abs = int(a1 * 100)
        angle2_abs = int(a2 * 100)

        data = struct.pack('>BHBH', d1, angle1_abs, d2, angle2_abs)
        self.target_angle1, self.target_angle2 = a1, a2

        if log_enabled:
            self.send_cmd(0x05, "臂体联合弯曲",
                          f"段1:[方向{d1}] {a1:.1f}°, 段2:[方向{d2}] {a2:.1f}°",
                          data, is_motor=True)
        else:
            frame = struct.pack('>BBB', 0xAA, 0x05, len(data)) + data
            frame += bytes([sum(frame) & 0xFF])
            self.worker.send_data(frame)

        self.update_ui()

    def direction_bend(self, section, dir_str):
        dir_map = {"up": 0, "right": 1, "down": 2, "left": 3}
        dir_idx = dir_map.get(dir_str, 0)
        if section == '1':
            self.dir1 = dir_idx
        else:
            self.dir2 = dir_idx

        self.update_ui()

    # ========== 闭环弯曲控制 ==========
    def send_closed_loop_bend_command(self):
        debug_mode = self.debug_check() if self.debug_check else False
        """启动/停止基于传感器反馈的电机循环控制"""
        if not self.is_started and not debug_mode:
            QMessageBox.warning(self, "错误", "请先点击启动控制系统")
            return
        if self.num_m == 0 or self.num_s == 0:
            QMessageBox.warning(self, "错误", "需要至少一个电机和一个传感器")
            return

        if not self.closed_loop_enabled:
            # 获取用户设定的目标位置（mm）
            self.cycle_target_motor_pos = self.spin_m_pos.spin.value()
            self.cycle_last_trigger = None
            self.closed_loop_enabled = True

            self.cycle_count = 0                     # 新增
            self.lbl_cycle_count.setText("循环次数: 0")  # 新增
            self.closed_loop_enabled = True

            self.btn_closed_bend.setText("⏹ 停止循环寿命检测")
            self.btn_closed_bend.set_normal_color("#D13438")
            # 禁用电机控制相关控件，防止手动干扰
            self.spin_m_pos.spin.setEnabled(False)
            self.spin_m_vel.spin.setEnabled(False)
            self.spin_m_acc.spin.setEnabled(False)
            self.btn_send_m.setEnabled(False)
            self.logger(f"🔄 启动电机循环控制，伸出目标位置={self.cycle_target_motor_pos} mm，收回位置=0 mm", port=self.port_name)
        else:
            # 停止循环
            self.closed_loop_enabled = False
            self.cycle_last_trigger = None
            self.btn_closed_bend.setText("循环寿命检测")
            self.btn_closed_bend.set_normal_color("#FF8C00")
            # 恢复电机控制控件
            self.spin_m_pos.spin.setEnabled(True)
            self.spin_m_vel.spin.setEnabled(True)
            self.spin_m_acc.spin.setEnabled(True)
            self.btn_send_m.setEnabled(True)
            self.logger("⏹ 停止循环寿命检测", port=self.port_name)

    def closed_loop_control(self):
        if not self.closed_loop_enabled:
            return
        debug_mode = self.debug_check() if self.debug_check else False
        if not self.is_started and not debug_mode:
            return
        if self.num_s == 0 or self.num_m == 0:
            return

        current_pos = self.motor_data[0][0] if self.motor_data else 0.0

        # ========== 1. 检查是否有正在等待确认的指令 ==========
        if self.pending_retry_target is not None:
            elapsed = time.time() - self.pending_send_time
            # 判断是否已到达目标（允许 0.1mm 误差）
            if abs(current_pos - self.pending_retry_target) <= 0.1:
                # 指令成功执行，清除等待状态
                self.logger(f"✅ 指令已到达目标 {self.pending_retry_target:.1f} mm", port=self.port_name)
                self.pending_retry_target = None
                # 清除后继续向下，可能立即满足阈值条件（例如刚刚收回完成，pitch 仍可能很高，但不会再触发同一方向，因为 cycle_last_trigger 已更新）
            elif elapsed >= 3.0:
                # 超过 3 秒且未到达 ⇒ 重发相同指令
                self.logger(f"⚠️ 指令超时未到达目标 {self.pending_retry_target:.1f} mm (当前{current_pos:.1f} mm)，重发", port=self.port_name)
                self._send_motor_command(self.pending_retry_target)
                self.pending_send_time = time.time()   # 重置计时器
                # 重发后本次结束，避免重复重发
                return
            else:
                # 未超时且未到达，继续等待，暂不进行新的阈值判断
                return

        # ========== 2. 无等待指令时，判断阈值条件 ==========
        current_pitch = self.sensor_data[0][0]  # 使用第一个 IMU 的 pitch 角度
        high_th = 10.0    # 收回阈值
        low_th = 4.0     # 伸出阈值

        # 高阈值触发：收回至 0 mm
        if current_pitch >= high_th and self.cycle_last_trigger != 'high':
            target = 0.0
            self._send_motor_command(target)
            self.pending_retry_target = target
            self.pending_send_time = time.time()
            # 在更新之前判断是否完成一个完整循环（之前是 low）
            if self.cycle_last_trigger == 'low':
                self.cycle_count += 1
                self.lbl_cycle_count.setText(f"循环次数: {self.cycle_count}")
                self.logger(f"循环次数累计至 {self.cycle_count}", port=self.port_name)
            self.cycle_last_trigger = 'high'
            self.logger(f"循环电机: pitch={current_pitch:.1f}° ≥ {high_th}°，发送收回至 {target} mm", port=self.port_name)

        # 低阈值触发：伸出至 -7 mm
        elif current_pitch <= low_th and self.cycle_last_trigger != 'low':
            target = -7.0   # 固定为 -7
            # mm，也可从界面获取：-self.spin_bend2.spin.value()
            self._send_motor_command(target)
            self.pending_retry_target = target
            self.pending_send_time = time.time()
            self.cycle_last_trigger = 'low'
            self.logger(f"循环电机: pitch={current_pitch:.1f}° ≤ {low_th}°，发送伸出至 {target} mm", port=self.port_name)
    def reset_cycle_count(self):
        self.cycle_count = 0
        self.lbl_cycle_count.setText("循环次数: 0")
        self.logger("循环计数已手动重置", port=self.port_name)

    def _send_motor_command(self, target_pos_mm):
        """控制电机移动到指定位置，不涉及等待确认状态"""
        if self.num_m == 0:
            return
        motor_id = 1
        vel = int(self.spin_m_vel.spin.value() * 100)
        acc = int(self.spin_m_acc.spin.value() * 100)
        direction = 0 if target_pos_mm >= 0 else 1
        distance = int(abs(target_pos_mm) * 100)
        data = struct.pack('>BBHHH', motor_id, direction, distance, vel, acc)
        self.send_cmd(0x03, f"循环电机{motor_id}",
                      f"位移:{target_pos_mm:.2f}, 速度:{vel/100:.2f}, 加速度:{acc/100:.2f}",
                      data, is_motor=True)
        if len(self.motor_target) >= motor_id:
            self.motor_target[motor_id-1] = [target_pos_mm, vel/100, acc/100]

    def apply_pid_params(self):
        self.pid.Kp = self.spin_kp.spin.value()
        self.pid.Ki = self.spin_ki.spin.value()
        self.pid.Kd = self.spin_kd.spin.value()
        self.logger(f"PID参数已更新: Kp={self.pid.Kp:.2f}, Ki={self.pid.Ki:.2f}, Kd={self.pid.Kd:.2f}", port=self.port_name)

    # ------------------ 核心：数据解析（调用后端）------------------
    @pyqtSlot(bytes)
    def parse_data(self, data):
        self.recv_buffer.extend(data)

        # 防止缓冲区无限增长
        if len(self.recv_buffer) > 4096:
            self.recv_buffer = self.recv_buffer[-2048:]

        i = 0
        while i < len(self.recv_buffer):
            # 查找帧头 0xBB,0x02
            if i + 3 > len(self.recv_buffer):
                break
            if self.recv_buffer[i] != 0xBB or self.recv_buffer[i+1] != 0x02:
                i += 1
                continue

            total_len = self.recv_buffer[i+2]   # 字节数
            # 帧总长度 = 3(头+命令+字节数) + total_len + 1(校验和)
            frame_total = total_len + 4

            if i + frame_total > len(self.recv_buffer):
                break   # 数据不足

            frame = bytes(self.recv_buffer[i:i+frame_total])

            # 校验和
            if (sum(frame[:-1]) & 0xFF) != frame[-1]:
                # 校验失败，跳过这个可能的帧头
                i += 1
                continue

            # 解析
            status = ProtocolParser.parse_frame(frame, apply_filter=True, filter_obj=self.data_filter)

            if status is not None:
                # 更新设备数量
                if status.num_motors != self.num_m or status.num_sensors != self.num_s:
                    self.num_m = status.num_motors
                    self.num_s = status.num_sensors
                    self.rebuild_cards()

                # 更新数据
                self.motor_data = [[m.pos, m.vel, m.acc] for m in status.motors]
                self.motor_states = [m.status for m in status.motors]
                self.sensor_data = [[s.pitch, s.roll, s.yaw] for s in status.sensors]
                self.current_angle1 = status.bend_angle1
                self.current_angle2 = status.bend_angle2

                # 低通滤波用于闭环
                self.filtered_bend_angle = (self.angle_filter_alpha * self.current_angle2 +
                                            (1 - self.angle_filter_alpha) * self.filtered_bend_angle)
                self.current_bend_angle = self.filtered_bend_angle

                # 可选：面积变化率（根据你的公式）
                if self.motor_data:
                    self.current_area_change = (1 - self.motor_data[0][0] / (2 * 3.1415926 * 50)) ** 2 * 100

                self.update_ui()

                if hasattr(self, 'spin_bend1') and not self.angle1_modified:
                    self.spin_bend1.spin.setValue(self.current_angle1)
                if hasattr(self, 'spin_bend2') and not self.angle2_modified:
                    self.spin_bend2.spin.setValue(self.current_angle2)

            # 处理完当前帧，移除缓冲区
            self.recv_buffer = self.recv_buffer[i+frame_total:]
            i = 0   # 重新从头扫描

    def update_ui(self):
        if len(self.cards_motor) != self.num_m or len(self.cards_sensor) != self.num_s:
            return
        for i in range(self.m_page * 3, min((self.m_page + 1) * 3, self.num_m)):
            labels = self.cards_motor[i][1]
            cur_pos_val, cur_vel_val, cur_acc_val = self.motor_data[i]
            target_pos, target_vel, target_acc = self.motor_target[i] if i < len(self.motor_target) else (0.0, 0.0, 0.0)
            state_val = self.motor_states[i]
            labels[0].setText(f"当前: {cur_pos_val:.2f}")
            labels[1].setText(f"目标: {target_pos:.2f}")
            labels[2].setText(f"当前: {cur_vel_val:.2f}")
            labels[3].setText(f"目标: {target_vel:.2f}")
            labels[4].setText(f"当前: {cur_acc_val:.2f}")
            labels[5].setText(f"目标: {target_acc:.2f}")
            lbl_state = labels[6]
            if state_val == 0:
                lbl_state.setStyleSheet("color: #D13438; font-size:10pt; border: none;")
            else:
                lbl_state.setStyleSheet("color: #107C10; font-size:10pt; border: none;")
        for i in range(self.s_page * 3, min((self.s_page + 1) * 3, self.num_s)):
            self.cards_sensor[i][1][0].setText(f"{self.sensor_data[i][0]:.2f}")
            self.cards_sensor[i][1][1].setText(f"{self.sensor_data[i][1]:.2f}")
            self.cards_sensor[i][1][2].setText(f"{self.sensor_data[i][2]:.2f}")

        # 定点专门监测更新
        idx = self.cb_view_id.currentIndex()
        is_motor = (self.cb_view_type.currentIndex() == 0)
        if is_motor:
            if 0 <= idx < self.num_m:
                values = self.motor_data[idx]
                for i, (_, value_label) in enumerate(self.single_cards):
                    value_label.setText(f"{values[i]:.2f}")
            else:
                for _, value_label in self.single_cards:
                    value_label.setText("--")
        else:
            if 0 <= idx < self.num_s:
                values = self.sensor_data[idx]
                for i, (_, value_label) in enumerate(self.single_cards):
                    value_label.setText(f"{values[i]:.2f}")
            else:
                for _, value_label in self.single_cards:
                    value_label.setText("--")

        if hasattr(self, 'cb_sensor_monitor'):
            self.update_sensor_monitor(self.cb_sensor_monitor.currentIndex())
        if hasattr(self, 'motor_status_ball') and hasattr(self, 'motor_states'):
            idx = self.cb_motor_id.currentIndex()
            if idx >= 0 and idx < len(self.motor_states):
                state_val = self.motor_states[idx]
                if state_val == 0:
                    self.motor_status_ball.setStyleSheet("color: #D13438; font-size: 8pt;")
                else:
                    self.motor_status_ball.setStyleSheet("color: #107C10; font-size: 8pt;")
        # 更新第一段角度卡片
        if hasattr(self, 'target_angle1_val'):
            self.target_angle1_val.setText(f"{self.target_angle1:.2f}")
        if hasattr(self, 'current_angle1_val'):
            self.current_angle1_val.setText(f"{self.current_angle1:.2f}")

        # 更新第二段角度卡片
        if hasattr(self, 'target_angle2_val'):
            self.target_angle2_val.setText(f"{self.target_angle2:.2f}")
        if hasattr(self, 'current_angle2_val'):
            self.current_angle2_val.setText(f"{self.current_angle2:.2f}")

    def update_motor_status_ball(self, idx=None):
        if idx is None:
            idx = self.cb_motor_id.currentIndex()
        if hasattr(self, 'motor_status_ball') and hasattr(self, 'motor_states'):
            if idx >= 0 and idx < len(self.motor_states):
                state_val = self.motor_states[idx]
                if state_val == 0:
                    self.motor_status_ball.setStyleSheet("color: #D13438; font-size: 8pt;")
                else:
                    self.motor_status_ball.setStyleSheet("color: #107C10; font-size: 8pt;")

    def update_sensor_monitor(self, idx=None):
        pass

    def record_history(self):
        if self.serial_error:
            return
        # 初始化起始时间（第一次调用时）
        if self.start_time is None:
            self.start_time = time.time()

        # 计算相对时间（秒，从 0 开始）
        current_time_sec = time.time() - self.start_time

        # 弯曲角度
        self.hist_bend_time.append(current_time_sec)
        self.hist_target1.append(self.target_angle1)
        self.hist_current1.append(self.current_angle1)
        self.hist_target2.append(self.target_angle2)
        self.hist_current2.append(self.current_angle2)

        # 限制长度（保留最近60秒）
        while len(self.hist_bend_time) > 0 and self.hist_bend_time[0] < current_time_sec - 60:
            self.hist_bend_time.pop(0)
            self.hist_target1.pop(0)
            self.hist_current1.pop(0)
            self.hist_target2.pop(0)
            self.hist_current2.pop(0)

        # 更新曲线窗口（如果已打开）
        if self.bend_graph_window and self.bend_graph_window.isVisible():
            self.bend_graph_controller.window.update_bend_data(
                self.hist_bend_time,
                self.hist_target1, self.hist_current1,
                self.hist_target2, self.hist_current2
            )

        # 电机/传感器数据
        should_record_motor = (self.num_m > 0 and self.motor_data and len(self.motor_data) == self.num_m)
        should_record_sensor = (self.num_s > 0 and self.sensor_data and len(self.sensor_data) == self.num_s)

        if should_record_motor or should_record_sensor:
            self.hist_time.append(current_time_sec)
            if should_record_motor:
                self.hist_motors.append(self.motor_data.copy())
            else:
                self.hist_motors.append([])
            if should_record_sensor:
                self.hist_sensors.append(self.sensor_data.copy())
            else:
                self.hist_sensors.append([])

            # 保持最近60s的点
            time_window = 60.0  # 秒
            while self.hist_time and self.hist_time[0] < current_time_sec - time_window:
                self.hist_time.pop(0)
                if self.hist_motors:
                    self.hist_motors.pop(0)
                if self.hist_sensors:
                    self.hist_sensors.pop(0)

        if hasattr(self, 'active_graph_controller') and self.active_graph_controller:
            # 检查对应的 UI 窗口是否可见
            if hasattr(self, 'active_graph_ui') and self.active_graph_ui and self.active_graph_ui.isVisible():
                if self.active_type == 'motor' and self.hist_motors and len(self.hist_motors) > 0:
                    valid_motor_data = [data for data in self.hist_motors if data and len(data) > 0]
                    valid_times = self.hist_time[-len(valid_motor_data):] if valid_motor_data else []
                    self.active_graph_controller.update_multi_data(valid_times, valid_motor_data)
                    if valid_motor_data and len(valid_motor_data) > 0:
                        self.active_graph_controller.update_multi_data(valid_times, valid_motor_data)
                elif self.active_type == 'sensor' and self.hist_sensors and len(self.hist_sensors) > 0:
                    valid_sensor_data = [data for data in self.hist_sensors if data and len(data) > 0]
                    valid_times = self.hist_time[-len(valid_sensor_data):] if valid_sensor_data else []
                    if valid_sensor_data and len(valid_sensor_data) > 0:
                        self.active_graph_controller.update_multi_data(valid_times, valid_sensor_data)

    def open_bend_graph(self):
        """打开弯曲角度历史曲线窗口"""
        if self.bend_graph_window is None:
            from UI.graph_window import BendGraphWindow
            from Core.GraphController import BendGraphController
            self.bend_graph_window = BendGraphWindow(self)
            self.bend_graph_controller = BendGraphController(self.bend_graph_window, self)
        self.bend_graph_window.show()
        # 立即使用四条曲线更新
        if self.hist_bend_time:
            self.bend_graph_controller.window.update_bend_data(
                self.hist_bend_time,
                self.hist_target1, self.hist_current1,
                self.hist_target2, self.hist_current2
            )

    #----------辅助函数----------------#
    def _create_custom_spinbox(self, min_val, max_val, default, prefix='', suffix='', step=1.0):
        """创建带自定义 +/- 按钮的 SpinBox 组合控件"""
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setValue(default)
        if prefix:
            spin.setPrefix(prefix)
        if suffix:
            spin.setSuffix(suffix)
        spin.setSingleStep(step)

        spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
        spin.setStyleSheet("""
            QDoubleSpinBox {
                min-height: 34px;
                font-size: 10pt;
                font-weight: bold;
                border: 2px solid #b0b0b0;
                border-radius: 10px;
                background: white;
                padding-right: 5px;
            }
            QDoubleSpinBox:focus {
                border-color: #0078D7;
            }
        """)

        btn_plus = QPushButton("+")
        btn_plus.setFixedSize(34, 34)
        btn_plus.setCursor(Qt.PointingHandCursor)
        btn_plus.setStyleSheet("""
            QPushButton {
                background-color: #f2f2f2;
                border: 2px solid #b0b0b0;
                border-radius: 5px;
                font-size: 10pt;
                font-weight: bold;
                color: #333;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
            QPushButton:pressed {
                background-color: #c0c0c0;
            }
        """)
        btn_plus.clicked.connect(lambda: spin.stepUp())

        btn_minus = QPushButton("−")
        btn_minus.setFixedSize(34, 34)
        btn_minus.setCursor(Qt.PointingHandCursor)
        btn_minus.setStyleSheet("""
            QPushButton {
                background-color: #f2f2f2;
                border: 2px solid #b0b0b0;
                border-radius: 5px;
                font-size: 10pt;
                font-weight: bold;
                color: #333;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
            QPushButton:pressed {
                background-color: #c0c0c0;
            }
        """)
        btn_minus.clicked.connect(lambda: spin.stepDown())

        layout.addWidget(spin)
        layout.addWidget(btn_plus)
        layout.addWidget(btn_minus)

        container.spin = spin
        return container

    def on_debug_changed(self, enabled):
        if enabled:
            # 创建虚拟电机（若不存在）
            if self.num_m == 0:
                self.expand_device('motor')
                self.logger("🐛 Debug 模式：已自动创建虚拟电机 ID:1")
            # 创建虚拟传感器（若不存在）
            if self.num_s == 0:
                self.expand_device('sensor')
                self.logger("🐛 Debug 模式：已自动创建虚拟传感器 ID:1")
        else:
            self.num_m = 0
            self.num_s = 0
            self.rebuild_cards()
        pass