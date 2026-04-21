# ==========================================
# 5. 设备选项卡
# ==========================================

# Qt类
import struct
import math
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QGridLayout,
                             QLabel, QComboBox, QDoubleSpinBox, QPushButton, QTabWidget,
                             QFrame, QSplitter, QMessageBox, QGraphicsDropShadowEffect)
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
    def __init__(self, port_name, parent_logger, auth_service=None):
        super().__init__()
        self.auth_service = auth_service #添加权限控制
        if self.auth_service and not self.auth_service.is_admin():
            # 普通用户可能没有某些高级操作权限
            # self.btn_cal.setEnabled(False)  # 例如禁用校准功能
            pass

        self.start_time = None   # 起始时间戳（None) 表示未初始化
        self.port_name, self.logger = port_name, parent_logger
        self.worker = SerialWorker(port_name)
        self.worker.signal_data.connect(self.parse_data)
        self.worker.signal_error.connect(self.handle_serial_error)
        self.serial_error = False

        self.recv_buffer = bytearray()
        self.is_started = False
        self.num_m, self.num_s = 0, 0
        self.motor_data, self.sensor_data = [], []
        self.motor_target = []
        self.motor_states = []
        self.scale_data = 100.0
        self.current_bend_angle = 0.0
        self.current_area_change = 0.0
        self.target_bend_angle = 0.0
        self.target_area_change = 0.0

        self.hist_bend_time = []       # 时间列表
        self.hist_bend_target = []     # 目标角度列表
        self.hist_bend_current = []    # 当前角度列表
        self.bend_graph_window = None  # 弯曲曲线窗口实例
        self.bend_graph_controller = None

        self.m_page, self.s_page = 0, 0
        self.cards_motor, self.cards_sensor = [], []

        self.plot_time = 0
        self.hist_time, self.hist_motors, self.hist_sensors = [], [], []
        self.active_graph = None

        # 创建滤波器
        self.data_filter = DataFilter(window_size=3)
        self.filtered_bend_angle = 0.0
        self.angle_filter_alpha = 0.3   # 滤波系数

        # 创建控制器
        self.last_sent_angle = None          # 记录上次闭环发送的目标角度
        self.angle_deadband = 1            # 死区阈值（度），变化小于此值时不发送
        self.closed_loop_enabled = False
        self.closed_loop_target_angle = 0.0
        self.pid = PID(Kp=1, Ki=0.01, Kd=0.01, dt=0.1, output_limits=(-70, 70), integral_limits=(-20, 20))
        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self.closed_loop_control)
        self.control_timer.start(100)   # 控制周期 100ms，与 dt 一致

        # 数据记录定时器
        self.history_timer = QTimer()
        self.history_timer.timeout.connect(self.record_history)
        self.history_timer.start(10) # 刷新率定义

        self.init_ui()
        self.worker.start()

    def init_ui(self):
        main_layout = QHBoxLayout(self)
        splitter = QSplitter(Qt.Horizontal)

        # 左侧面板
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)

        g_power = QGroupBox("1. 系统操作权限")
        l_power = QHBoxLayout(g_power)


        self.btn_toggle = AnimatedButton("▶ 启动控制系统","#107C10", "#063A06")
        self.btn_toggle.setCheckable(True) # 设置为可选中状态(开关模式)
        # self.btn_toggle.setFixedSize(300, 100)

        shadow = QGraphicsDropShadowEffect()

        # 连接后状态改变为触发
        self.btn_toggle.toggled.connect(self.sys_toggle)
        self.btn_toggle.setGraphicsEffect(shadow)

        self.btn_stop = AnimatedButton("⏹紧急停止","red", "#A80000")
        shadow = QGraphicsDropShadowEffect()
        # shadow.setBlurRadius(15)      # 阴影模糊半径
        # shadow.setOffset(20, 20)      # 阴影偏移量 (X, Y)
        #shadow.setColor(Qt.black)      # 阴影颜色
        self.btn_stop.setGraphicsEffect(shadow)
        self.btn_stop.setProperty("class", "emergency")     # 使用自定义属性 emergency
        self.btn_stop.clicked.connect(self.sys_stop)

        l_power.addWidget(self.btn_toggle)
        l_power.addWidget(self.btn_stop)
        left_layout.addWidget(g_power)

        g_quick = QGroupBox("2. 弯曲与截面收缩控制")
        l_quick = QVBoxLayout(g_quick)
        self.btn_home = AnimatedButton("⌂ 一键归中","#1E1E1E","#505050")
        self.btn_home.clicked.connect(self.send_home_command)
        l_shrink = QHBoxLayout()
        self.spin_scale = QDoubleSpinBox()
        self.spin_scale.setRange(75, 100)
        self.spin_scale.setValue(75)
        self.btn_shrink = AnimatedButton("⇲ 截面收缩","#00BCD4","#505050")
        self.btn_shrink.clicked.connect(self.send_scale_command)
        l_shrink.addWidget(QLabel("Scale(%):"))
        l_shrink.addWidget(self.spin_scale)
        l_shrink.addWidget(self.btn_shrink)

        # 原弯曲控制布局
        l_bend = QHBoxLayout()
        self.spin_bend = QDoubleSpinBox()
        self.spin_bend.setRange(-70, 70)
        self.spin_bend.setValue(0)
        self.btn_bend = AnimatedButton("开环弯曲","#00BCD4","#505050")
        self.btn_bend.clicked.connect(lambda checked: self.send_bend_command())

        # 新增闭环弯曲按钮
        self.btn_closed_bend = AnimatedButton("闭环弯曲","#FF8C00","#B85C00")  # 橙色风格
        self.btn_closed_bend.clicked.connect(self.send_closed_loop_bend_command)

        h_pid = QHBoxLayout()
        h_pid.addWidget(QLabel("Kp:"))
        self.spin_kp = QDoubleSpinBox(); self.spin_kp.setRange(0, 10); self.spin_kp.setValue(0.5)
        h_pid.addWidget(self.spin_kp)
        h_pid.addWidget(QLabel("Ki:"))
        self.spin_ki = QDoubleSpinBox(); self.spin_ki.setRange(0, 10); self.spin_ki.setValue(0.1)
        h_pid.addWidget(self.spin_ki)
        h_pid.addWidget(QLabel("Kd:"))
        self.spin_kd = QDoubleSpinBox(); self.spin_kd.setRange(0, 10); self.spin_kd.setValue(0.05)
        h_pid.addWidget(self.spin_kd)
        btn_apply_pid = AnimatedButton("应用PID参数", "#1E1E1E","#505050")
        btn_apply_pid.clicked.connect(self.apply_pid_params)
        h_pid.addWidget(btn_apply_pid)
        l_quick.addLayout(h_pid)


        l_bend.addWidget(QLabel("Angle(deg):"))
        l_bend.addWidget(self.spin_bend)
        l_bend.addWidget(self.btn_bend)
        l_bend.addWidget(self.btn_closed_bend)   # 添加新按钮
        l_quick.addWidget(self.btn_home)
        l_quick.addLayout(l_shrink)
        l_quick.addLayout(l_bend)
        left_layout.addWidget(g_quick)

        g_addr = QGroupBox("3. 电机控制")
        f_addr = QGridLayout(g_addr)
        self.cb_motor_id = QComboBox()
        self.spin_m_pos = QDoubleSpinBox()
        self.spin_m_pos.setRange(-500, 500)
        self.spin_m_pos.setPrefix("位移: ")
        self.spin_m_pos.setValue(40)
        self.spin_m_pos.setSuffix(" mm")
        self.spin_m_vel = QDoubleSpinBox()
        self.spin_m_vel.setRange(-500, 500)
        self.spin_m_vel.setPrefix("速度: ")
        self.spin_m_vel.setValue(10)
        self.spin_m_vel.setSuffix(" mm/s")
        self.spin_m_acc = QDoubleSpinBox()
        self.spin_m_acc.setRange(-500, 500)
        self.spin_m_acc.setPrefix("加速度: ")
        self.spin_m_acc.setValue(0)
        self.spin_m_acc.setSuffix(" mm/s^2")
        self.btn_send_m =  AnimatedButton("发至电机","#00BCD4","#505050")
        self.btn_send_m.clicked.connect(self.send_motor)
        f_addr.addWidget(QLabel("电机ID:"), 0, 0)
        f_addr.addWidget(self.cb_motor_id, 0, 1)
        f_addr.addWidget(self.btn_send_m, 0, 2)
        self.motor_status_ball = QLabel("●")
        self.motor_status_ball.setStyleSheet("color: red; font-size: 8pt;")
        f_addr.addWidget(self.motor_status_ball, 0, 3)
        self.cb_motor_id.currentIndexChanged.connect(self.update_motor_status_ball)
        f_addr.addWidget(self.spin_m_pos, 1, 0)
        f_addr.addWidget(self.spin_m_vel, 1, 1)
        f_addr.addWidget(self.spin_m_acc, 1, 2)
        left_layout.addWidget(g_addr)

        g_sensor = QGroupBox("4. IMU数据监控")
        l_sensor = QVBoxLayout(g_sensor)
        self.cb_sensor_monitor = QComboBox()
        self.cb_sensor_monitor.currentIndexChanged.connect(self.update_sensor_monitor)
        h_sensor_line = QHBoxLayout()
        h_sensor_line.addWidget(QLabel("IMU ID:"))
        h_sensor_line.addWidget(self.cb_sensor_monitor)
        l_sensor.addLayout(h_sensor_line)
        self.btn_cal = AnimatedButton("IMU校准", "#1E1E1E","#505050")
        self.btn_cal.clicked.connect(self.calibrate_sensor)
        self.btn_read = AnimatedButton("读取IMU数据","#1E1E1E","#505050")
        self.btn_read.clicked.connect(self.read_sensor_data)
        l_sensor.addWidget(self.btn_cal)
        l_sensor.addWidget(self.btn_read)
        left_layout.addWidget(g_sensor)
        left_layout.addStretch()

        # 右侧看板
        right_widget = QWidget()
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
        self.lbl_s_page = QLabel("IMU 1/1 页")
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
        self.tabs.addTab(tab_all, "👁 电机与IMU数据监控")

        tab_bend = QWidget()
        v_bend = QVBoxLayout(tab_bend)

        # --- 第一行：目标弯曲角度 + 当前弯曲角度 ---
        hbox_angles = QHBoxLayout()

        self.target_angle_card, self.target_angle_val = self.create_flat_card(
            "目标弯曲角度(deg)", "0.00", "#D13438"
        )
        hbox_angles.addWidget(self.target_angle_card)

        self.current_angle_card, self.current_angle_val = self.create_flat_card(
            "当前弯曲角度(deg)", "0.00", "#D13438"
        )
        hbox_angles.addWidget(self.current_angle_card)

        v_bend.addLayout(hbox_angles)

        # --- 第二行：目标喷嘴面积 + 当前喷嘴面积 ---
        hbox_area = QHBoxLayout()

        self.target_area_card, self.target_area_val = self.create_flat_card(
            "目标喷嘴截面面积缩放比(%)", "0.00", "#107C10"
        )
        hbox_area.addWidget(self.target_area_card)

        self.current_area_card, self.current_area_val = self.create_flat_card(
            "当前喷嘴截面面积缩放比(%)", "0.00", "#107C10"
        )
        hbox_area.addWidget(self.current_area_card)

        v_bend.addLayout(hbox_area)

        self.tabs.addTab(tab_bend, "🔧 LQTS喷管运动数据监控")

        # 定点专门监测页面（修改部分）
        tab_single = QWidget()
        v_single = QVBoxLayout(tab_single)
        h_sel = QHBoxLayout()
        self.cb_view_type = QComboBox()
        self.cb_view_type.addItems(["定点监测: 电机", "定点监测: IMU"])
        self.cb_view_type.currentIndexChanged.connect(self.update_single_monitor_labels)
        self.cb_view_id = QComboBox()
        self.cb_view_id.currentIndexChanged.connect(lambda: self.update_ui())
        h_sel.addWidget(self.cb_view_type)
        h_sel.addWidget(self.cb_view_id)
        h_sel.addStretch()
        v_single.addLayout(h_sel)
        # 创建动态卡片
        self.single_cards = []  # (title_label, value_label)
        for default_title, default_color in [("位移 (mm)", "#D13438"), ("速度 (mm/s)", "#107C10"), ("加速度 (mm/s²)", "#0078D7")]:
            card_frame = QFrame()
            card_frame.setStyleSheet("QFrame { background: #d9d9d6; border: 3px solid white; border-radius: 10px; }")
            card_layout = QHBoxLayout(card_frame)
            title_label = QLabel(default_title)
            title_label.setStyleSheet("color: black; font-weight:bold; border:none; font-size:15pt;")
            value_label = QLabel("0.00")
            value_label.setStyleSheet(f"color: {default_color}; font-size: 15pt; font-weight: bold; border: none;")
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            card_layout.addWidget(title_label)
            card_layout.addStretch()
            card_layout.addWidget(value_label)
            v_single.addWidget(card_frame)
            self.single_cards.append((title_label, value_label))
        self.tabs.addTab(tab_single, "🎯 定点监测(电机与IMU)")

        self.right_layout.addWidget(self.tabs)
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 3)
        main_layout.addWidget(splitter)
        self.rebuild_cards()

    def update_single_monitor_labels(self):
        """根据定点监测类型更新卡片标题、颜色以及ID下拉框选项"""
        is_motor = (self.cb_view_type.currentIndex() == 0)
        if is_motor:
            titles = ["位移 (mm)", "速度 (mm/s)", "加速度 (mm/s²)"]
            colors = ["#D13438", "#107C10", "#0078D7"]
            # 更新ID下拉框选项为电机ID
            self.update_single_monitor_ids(range(1, self.num_m + 1))
        else:
            titles = ["Pitch (deg)", "Roll (deg)", "Yaw (deg)"]
            colors = ["#D13438", "#107C10", "#0078D7"]
            # 更新ID下拉框选项为IMU ID
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
                background: #d9d9d6; 
                border: 1px solid white; 
                border-radius: 6px;
            }
            QFrame#motorCard:hover {
                border: 2px solid white;
            }
        """)
        main_layout = QVBoxLayout(frame)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(10)
        top_widget = QWidget()
        top_layout = QHBoxLayout(top_widget)
        top_layout.setContentsMargins(0, 0, 0, 0)
        title_label = QLabel(title)
        title_label.setStyleSheet("color: #333; font-weight: bold; font-size: 10pt; border: none;")
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
            title_lbl.setStyleSheet(f"background-color: #d9d9d6;color: {color}; font-size: 10pt; font-weight: bold; border: none;")
            title_lbl.setAlignment(Qt.AlignCenter)
            block_layout.addWidget(title_lbl)
            value_widget = QWidget()
            value_layout = QHBoxLayout(value_widget)
            value_layout.setContentsMargins(0, 0, 0, 0)
            value_layout.setSpacing(30)
            cur_label = QLabel("当前: 0.00")
            cur_label.setStyleSheet("color: #0078D7; font-size: 8pt; font-weight: bold; border: none;")
            cur_label.setAlignment(Qt.AlignCenter)
            tar_label = QLabel("目标: 0.00")
            tar_label.setStyleSheet("color: #666; font-size: 8pt; border: none;")
            tar_label.setAlignment(Qt.AlignCenter)
            value_layout.addStretch()
            value_layout.addWidget(cur_label)
            value_layout.addWidget(tar_label)
            value_layout.addStretch()
            block_layout.addWidget(value_widget)
            return block_widget, cur_label, tar_label

        block_pos, cur_pos, tar_pos = create_block("位移", "mm", "#D13438")
        block_vel, cur_vel, tar_vel = create_block("速度", "mm/s", "#D13438")
        block_acc, cur_acc, tar_acc = create_block("加速度", "mm/s²", "#D13438")
        main_layout.addWidget(block_pos)
        line1 = QFrame()
        line1.setFrameShape(QFrame.HLine)
        line1.setStyleSheet("background-color: white; border: none; height: 3px;")
        main_layout.addWidget(line1)
        main_layout.addWidget(block_vel)
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setStyleSheet("background-color: white; border: none; height: 3px;")
        main_layout.addWidget(line2)
        main_layout.addWidget(block_acc)
        lbls = [cur_pos, tar_pos, cur_vel, tar_vel, cur_acc, tar_acc, state_ball]
        return frame, lbls

    def create_sensor_card(self, title, color):
        frame = QFrame()
        frame.setObjectName("sensorCard")
        frame.setStyleSheet("""
            QFrame#sensorCard { 
                background: #d9d9d6; 
                border: 1px solid white; 
                border-radius: 6px;
            }
            QFrame#sensorCard:hover {
                border: 2px solid white;
            }
        """)
        main_layout = QVBoxLayout(frame)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(8)
        top_widget = QWidget()
        top_layout = QHBoxLayout(top_widget)
        top_layout.setContentsMargins(0, 0, 0, 0)
        title_label = QLabel(title)
        title_label.setStyleSheet("color: #333; font-weight: bold; font-size: 10pt; border: none;")
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
            value_label.setStyleSheet(f"color: #000; font-size: 8pt; font-weight: bold; border: none;")
            value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            block_layout.addWidget(value_label)
            block_layout.addStretch()
            return block_widget, value_label

        block_pitch, val_pitch = create_axis_block("Pitch", "deg")
        block_roll, val_roll = create_axis_block("Roll", "deg")
        block_yaw, val_yaw = create_axis_block("Yaw", "deg")
        main_layout.addWidget(block_pitch)
        line1 = QFrame()
        line1.setFrameShape(QFrame.HLine)
        line1.setStyleSheet("background-color: white; border: none; height: 1px;")
        main_layout.addWidget(line1)
        main_layout.addWidget(block_roll)
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setStyleSheet("background-color: white; border: none; height: 1px;")
        main_layout.addWidget(line2)
        main_layout.addWidget(block_yaw)
        lbls = [val_pitch, val_roll, val_yaw]
        return frame, lbls

    def create_flat_card(self, title, val, color):
        frame = QFrame()
        frame.setStyleSheet("QFrame { background: #d9d9d6; border: 3px solid white; border-radius: 6px; }")
        layout = QHBoxLayout(frame)
        lbl_val = QLabel(val)
        lbl_val.setStyleSheet(f"color: {color}; font-size: 12pt; font-weight: bold; border: none;")
        layout.addWidget(QLabel(title, styleSheet="color: black; font-weight:bold; border:none; font-size:12pt;"))
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
            self.sensor_data.append([0.0, 0.0, 0.0])
            self.hist_sensors = [step + [[0.0, 0.0, 0.0]] for step in self.hist_sensors]
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
            card, lbls = self.create_sensor_card(f"IMU ID:{i + 1}", "#D83B01")
            self.cards_sensor.append((card, lbls))
            self.grid_s.addWidget(card, 0, i % 3)
            self.cb_sensor_monitor.addItem(f"IMU {i + 1}")
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
        self.lbl_s_page.setText(f"IMU {self.s_page + 1}/{s_pages} 页")
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
        self.send_cmd(0x00, "失能", "关闭LQTS喷管", is_motor=True)

    def sys_start(self):
        if self.is_started:
            return
        self.is_started = True

        self.send_cmd(0x01, "使能", "启动LQTS喷管", is_motor=True)

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
        for btn in [self.btn_stop, self.btn_home, self.btn_m_next, self.btn_m_prev,
                    self.btn_s_next, self.btn_s_prev, self.btn_send_m, self.btn_bend,
                    self.btn_shrink, self.btn_cal, self.btn_read]:
            btn.setEnabled(False)

    def send_cmd(self, func_code, action, detail, data=b'', is_motor=True):
        try:
            if func_code not in [0x00, 0x01, 0x02, 0x04, 0x06, 0xFE] and not self.is_started:
                error_msg = "请先点击启动控制系统"
                QMessageBox.warning(self, "拒绝", error_msg)
                self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
                return

            frame_head = 0xAA if is_motor else 0xBB
            frame = struct.pack('>BBB', frame_head, func_code, len(data)) + data
            frame += bytes([sum(frame) & 0xFF])
            self.worker.send_data(frame)
            # 记录操作历史
            GlobalHistory.add_record(self.port_name, action, detail, frame.hex().upper())

            # 根据功能码选择不同的日志级别
            if func_code == 0x02:  # 紧急停止 - 使用 WARNING 级别
                self.logger(f"📤 {action} -> {detail}", raw_data=frame, level="WARNING", port=self.port_name)
            else:  # 其他操作 - 使用 INFO 级别
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
        # 检查是否有电机
        if self.num_m == 0:
            error_msg = "当前没有可用的电机设备，无法进行电机控制"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        m_id = self.cb_motor_id.currentIndex() + 1

        # 检查电机ID是否有效
        if m_id > self.num_m:
            error_msg = f"电机ID {m_id} 无效，当前只有 {self.num_m} 个电机"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        try:
            pos = int(self.spin_m_pos.value() * 100)
            vel = int(self.spin_m_vel.value() * 100)
            acc = int(self.spin_m_acc.value() * 100)

            # 更新目标值（确保列表长度足够）
            while len(self.motor_target) < self.num_m:
                self.motor_target.append([0.0, 0.0, 0.0])

            if m_id <= len(self.motor_target):
                self.motor_target[m_id-1] = [self.spin_m_pos.value(), self.spin_m_vel.value(), self.spin_m_acc.value()]

            self.update_ui()

            direction = 0 if pos >= 0 else 1
            distance = abs(pos)
            data = struct.pack('>BBHHH', m_id, direction, distance, vel, acc)
            self.send_cmd(0x03, f"控制电机{m_id}", f"位移:{pos/100}, 速度:{vel/100}, 加速度:{acc/100}", data, is_motor=True)

        except Exception as e:
            error_msg = f"发送电机控制命令失败: {str(e)}"
            QMessageBox.critical(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)

    def calibrate_sensor(self):
        if self.num_s == 0:
            error_msg = "当前没有可用的IMU传感器，无法进行传感器校准"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        idx = self.cb_sensor_monitor.currentIndex() + 1
        if idx > self.num_s:
            error_msg = f"IMU ID {idx} 无效，当前只有 {self.num_s} 个传感器"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        try:
            self.send_cmd(0x03, f"校准IMU{idx}", f"Sensor {idx} 校准", struct.pack('>B', idx), is_motor=False)
        except Exception as e:
            error_msg = f"发送IMU校准命令失败: {str(e)}"
            QMessageBox.critical(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)

    def read_sensor_data(self):
        if self.num_s == 0:
            error_msg = "当前没有可用的IMU传感器，无法读取传感器"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        idx = self.cb_sensor_monitor.currentIndex() + 1
        if idx > self.num_s:
            error_msg = f"IMU ID {idx} 无效，当前只有 {self.num_s} 个传感器"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        try:
            self.send_cmd(0x01, f"读取IMU{idx}", f"请求IMU{idx}数据", struct.pack('>B', idx), is_motor=False)
        except Exception as e:
            error_msg = f"发送读取IMU数据命令失败: {str(e)}"
            QMessageBox.critical(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)

    def send_home_command(self):
        if not self.is_started:
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
            # 喷管目标弯曲角度置 0
            self.target_bend_angle = 0
        except Exception as e:
            error_msg = f"发送一键归中命令失败: {str(e)}"
            QMessageBox.critical(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)

    def send_scale_command(self):
        if not self.is_started:
            error_msg = "请先点击启动控制系统"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        if self.num_m == 0:
            error_msg = "当前没有可用的电机设备,无法进行截面收缩"
            QMessageBox.warning(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)
            return

        try:
            self.target_area_change = int(self.spin_scale.value())
            count = 1
            special_addr = 0xFD
            direction = 1
            data = struct.pack('>BBBH', count, special_addr, direction, self.target_area_change* 100)
            self.send_cmd(0x06, "截面收缩", f"收缩比例={self.target_area_change}%", data, is_motor=True)
        except Exception as e:
            error_msg = f"发送截面收缩命令失败: {str(e)}"
            QMessageBox.critical(self, "错误", error_msg)
            self.logger(f"❌ {error_msg}", level="ERROR", port=self.port_name)

    def send_bend_command(self, angle_deg=None, log_enabled=True):
        """
        发送弯曲命令（开环或闭环均可调用）
        :param angle_deg: 目标角度（度），若为 None 则从 spin_bend 取值
        :param log_enabled: 是否记录日志（闭环控制时可设为 False）
        """
        if not self.is_started:
            if log_enabled:
                QMessageBox.warning(self, "错误", "请先点击启动控制系统")
            return
        if self.num_m == 0:
            if log_enabled:
                QMessageBox.warning(self, "错误", "当前没有可用的电机设备，无法进行弯曲")
            return

        # 确定目标角度
        if angle_deg is None:
            target_angle = self.spin_bend.value()
        else:
            target_angle = angle_deg

        # 更新界面显示的目标值（开环时显示 spin_bend 值，闭环时显示实际目标）
        self.target_bend_angle = target_angle

        direction = 0 if target_angle >= 0 else 1
        angle = abs(int(target_angle * 100))   # 转为整数（0.01度单位）

        count = 1
        special_addr = 0xFE
        data = struct.pack('>BBBH', count, special_addr, direction, angle)

        action = "喷管弯曲"
        detail = f"方向:{'正' if direction == 0 else '负'}, 角度:{angle/100}度"
        self.send_cmd(0x06, action, detail, data, is_motor=True)

    def send_closed_loop_bend_command(self):
        """启动/停止闭环弯曲控制"""
        if not self.is_started:
            QMessageBox.warning(self, "错误", "请先点击启动控制系统")
            return
        if self.num_m == 0 or self.num_s == 0:
            QMessageBox.warning(self, "错误", "需要至少一个电机和一个 IMU 才能进行闭环弯曲控制")
            return

        if not self.closed_loop_enabled:
            # 启动闭环控制
            self.closed_loop_target_angle = self.spin_bend.value()
            self.pid.reset()
            self.last_sent_angle = None          # 重置记录
            self.closed_loop_enabled = True
            self.btn_closed_bend.setText("⏹ 停止闭环弯曲")
            self.btn_closed_bend.set_normal_color("#D13438")
            self.logger(f"🔄 启动闭环弯曲控制，目标角度={self.closed_loop_target_angle}°", port=self.port_name)
        else:
            # 停止闭环控制
            self.closed_loop_enabled = False
            self.btn_closed_bend.setText("闭环弯曲")
            self.btn_closed_bend.set_normal_color("#FF8C00")
            self.logger("⏹ 停止闭环弯曲控制", port=self.port_name)

    def closed_loop_control(self):
        if not self.closed_loop_enabled or not self.is_started:
            return
        if self.num_s == 0:
            return

        current_angle = self.filtered_bend_angle   # 使用滤波值
        # PID 输出即为目标角度（度）
        target_angle = -self.pid.update(self.closed_loop_target_angle, current_angle)
        # 限幅
        target_angle = max(-70, min(70, target_angle))

        # 死区判断：如果与上次发送的角度差异小于阈值，则不发送
        if self.last_sent_angle is not None:
            if abs(target_angle - self.last_sent_angle) < self.angle_deadband:
                return

        # 发送弯曲命令（不记录日志）
        self.send_bend_command(angle_deg=target_angle, log_enabled=False)
        self.last_sent_angle = target_angle

    def apply_pid_params(self):
        self.pid.Kp = self.spin_kp.value()
        self.pid.Ki = self.spin_ki.value()
        self.pid.Kd = self.spin_kd.value()
        self.logger(f"PID参数已更新: Kp={self.pid.Kp}, Ki={self.pid.Ki}, Kd={self.pid.Kd}", port=self.port_name)

    # ------------------ 核心：数据解析（调用后端）------------------
    @pyqtSlot(bytes)
    def parse_data(self, data):
        """接收串口原始数据，组帧并调用后端解析器"""
        self.recv_buffer.extend(data)
        if len(self.recv_buffer) > 1024:
            self.recv_buffer.clear()
            return

        while len(self.recv_buffer) >= 5:
            # 查找帧头 0xBB
            if self.recv_buffer[0] != 0xBB:
                self.recv_buffer.pop(0)
                continue

            d_len = self.recv_buffer[2]
            if d_len > 255:
                self.recv_buffer.pop(0)
                continue

            frame_len = 3 + d_len + 1
            if len(self.recv_buffer) < frame_len:
                break

            # 取出完整帧
            frame = bytes(self.recv_buffer[:frame_len])
            self.recv_buffer = self.recv_buffer[frame_len:]

            # 校验和
            if (sum(frame[:-1]) & 0xFF) != frame[-1]:
                continue   # 校验失败，丢弃该帧

            # 调用后端解析器（启用滤波）
            status = ProtocolParser.parse_frame(
                frame,
                apply_filter=True,          # 启用滤波
                filter_obj=self.data_filter
            )

            if status is None:
                continue   # 解析失败（非 0x02 帧或数据不足）

            # ---------- 根据解析结果更新前端状态 ----------
            # 1. 若设备数量变化，重建卡片
            if status.num_motors != self.num_m or status.num_sensors != self.num_s:
                self.num_m = status.num_motors
                self.num_s = status.num_sensors
                self.rebuild_cards()

            # 2. 更新电机和传感器数据（格式转换为原前端使用的列表）
            self.motor_data = [[m.pos, m.vel, m.acc] for m in status.motors]
            self.motor_states = [m.status for m in status.motors]
            self.sensor_data = [[s.pitch, s.roll, s.yaw] for s in status.sensors]

            # 3. 更新喷管参数
            # ---------- 修正：更新数值变量，不要覆盖 QLabel 对象 ----------
            if self.num_s > 0:
                # 假设第一个 IMU 的 pitch 角度代表当前弯曲角度（根据实际情况调整）
                self.current_bend_angle = self.sensor_data[0][0]
            else:
                self.current_bend_angle = 0.0
            # 更新 self.current_bend_angle，使用一阶低通滤波
            self.filtered_bend_angle = self.angle_filter_alpha * self.current_bend_angle + (1 - self.angle_filter_alpha) * self.filtered_bend_angle
            self.current_bend_angle = self.filtered_bend_angle   # 或保留原值用于显示，闭环用滤波值
            # 当前面积缩放比（根据实际协议赋值）
            self.current_area_change = (1-self.motor_data[0][0] / (2 * 3.1415926 * 50))*(1-self.motor_data[0][0] / (2 * 3.1415926 * 50))*100

            # 4. 刷新界面
            self.update_ui()

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
        if hasattr(self, 'target_angle_val'):
            self.target_angle_val.setText(f"{self.target_bend_angle:.2f}")
        if hasattr(self, 'current_angle_val'):
            self.current_angle_val.setText(f"{self.current_bend_angle:.2f}")
        if hasattr(self, 'target_area_val'):
            self.target_area_val.setText(f"{self.target_area_change:.2f}")
        if hasattr(self, 'current_area_val'):
            self.current_area_val.setText(f"{self.current_area_change:.2f}")

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
        self.hist_bend_target.append(self.target_bend_angle)
        self.hist_bend_current.append(self.current_bend_angle)

        # 限制长度（保留最近60秒）
        while len(self.hist_bend_time) > 0 and self.hist_bend_time[0] < current_time_sec - 60:
            self.hist_bend_time.pop(0)
            self.hist_bend_target.pop(0)
            self.hist_bend_current.pop(0)

        # 更新曲线窗口（如果已打开）
        if self.bend_graph_window and self.bend_graph_window.isVisible():
            self.bend_graph_controller.window.update_data(
                self.hist_bend_time, self.hist_bend_target, self.hist_bend_current
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
        self.bend_graph_window.raise_()
        # 立即更新数据
        if self.hist_bend_time:
            self.bend_graph_controller.window.update_data(
                self.hist_bend_time, self.hist_bend_target, self.hist_bend_current
            )
