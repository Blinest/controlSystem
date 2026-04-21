# ui/graph_window.py

# Qt类
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLabel, QCheckBox, QWidget, QScrollArea, QApplication,
                             QDoubleSpinBox, QListWidget, QListWidgetItem)
from PyQt5.QtCore import pyqtSignal, Qt

# 工具类
import pyqtgraph as pg
import csv
from datetime import datetime
import os


class GraphWindowUI(QDialog):
    def __init__(self, title, is_motor=True, num_devices=1, parent=None, enable_auto_save=True, is_history_mode=False):
        super().__init__(parent)
        self.setWindowTitle(title)
        # 根据分辨率自动动态设置窗口大小
        screen = QApplication.primaryScreen().availableGeometry()
        self.resize(int(screen.width() * 0.6), int(screen.height() * 0.7))
        self.setMinimumSize(640, 480)   # 防止缩得过小导致控件挤爆
        self.showNormal()               # 确保不是最大化状态
        self.is_motor = is_motor

        # 主布局
        self.layout = QVBoxLayout(self)

        # --- 控制按钮行 ---
        self.btn_focus = QPushButton("🎯 一键聚焦")
        self.btn_focus_x = QPushButton("📏 X轴聚焦")
        self.btn_focus_y = QPushButton("📐 Y轴聚焦")
        self.btn_reset = QPushButton("🔄 重置视图")
        self.btn_save = QPushButton("💾 保存数据")
        self.btn_load = QPushButton("📂 加载历史")

        self.btn_focus.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.btn_focus_x.setStyleSheet("background-color: #2196F3; color: white;")
        self.btn_focus_y.setStyleSheet("background-color: #FF9800; color: white;")
        self.btn_reset.setStyleSheet("background-color: #9E9E9E; color: white;")
        self.btn_save.setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold;")
        self.btn_load.setStyleSheet("background-color: #FF5722; color: white; font-weight: bold;")

        focus_layout = QHBoxLayout()
        focus_layout.addWidget(self.btn_focus)
        focus_layout.addWidget(self.btn_focus_x)
        focus_layout.addWidget(self.btn_focus_y)
        focus_layout.addWidget(self.btn_reset)
        focus_layout.addWidget(self.btn_save)
        focus_layout.addWidget(self.btn_load)

        ctrl_layout = QHBoxLayout()
        ctrl_layout.addLayout(focus_layout)
        ctrl_layout.addStretch()
        self.layout.addLayout(ctrl_layout)

        # 如果是历史模式，隐藏保存和加载按钮
        if not is_history_mode:
            focus_layout.addWidget(self.btn_save)
            focus_layout.addWidget(self.btn_load)
        else:
            self.btn_save.hide()
            self.btn_load.hide()

        if enable_auto_save:
            auto_save_layout = QHBoxLayout()
            self.chk_auto_save = QCheckBox("⏱ 自动保存")
            self.spin_interval = QDoubleSpinBox()
            self.spin_interval.setRange(5.0, 3600.0)
            self.spin_interval.setValue(30.0)
            self.spin_interval.setSuffix(" 秒")
            self.spin_interval.setEnabled(False)
            self.chk_auto_save.toggled.connect(self.spin_interval.setEnabled)
            auto_save_layout.addWidget(self.chk_auto_save)
            auto_save_layout.addWidget(QLabel("间隔:"))
            auto_save_layout.addWidget(self.spin_interval)
            auto_save_layout.addStretch()
            self.layout.addLayout(auto_save_layout)
        else:
            # 历史窗口无自动保存控件，创建占位变量避免后续访问出错
            self.chk_auto_save = None
            self.spin_interval = None

        # --- 绘图区域 ---
        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.PlotWidget(background='w')
        self.plot_widget.addLegend()
        self.plot_widget.setLabel('bottom', '时间', units='秒')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLimits(xMin=0)
        self.plot_widget.setMenuEnabled(True)
        self.plot_widget.scene().contextMenu = self._create_context_menu_wrapper
        self.layout.addWidget(self.plot_widget)

        #--- 坐标显示标签 ---
        self.coord_label = QLabel("点击曲线查看该点坐标")
        self.coord_label.setStyleSheet("""
            background-color: #F5F5F5;
            border: 1px solid #CCC;
            border-radius: 4px;
            padding: 5px 10px;
            font-family: 'Consolas', monospace;
            font-size: 10pt;
        """)
        self.coord_label.setMinimumHeight(60)      # 设置最小高度
        self.coord_label.setWordWrap(True)         # 允许换行
        self.layout.addWidget(self.coord_label)

        # --- 曲线与复选框 ---
        self.curves = []
        self.checkboxes = []
        self.chk_layout = QHBoxLayout()
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
                  '#9467bd', '#8c564b', '#e377c2', '#7f7f7f']
        prefix = "电机" if is_motor else "传感器"

        for i in range(num_devices):
            chk = QCheckBox(f"{prefix} {i + 1}")
            chk.setChecked(True)
            color = colors[i % len(colors)]
            chk.setStyleSheet(f"color: {color}; font-weight: bold;")
            self.chk_layout.addWidget(chk)
            self.checkboxes.append(chk)

            name_x = f"{prefix}{i + 1}-位移" if is_motor else f"{prefix}{i + 1}-偏航X"
            name_y = f"{prefix}{i + 1}-速度" if is_motor else f"{prefix}{i + 1}-偏航Y"
            name_z = f"{prefix}{i + 1}-加速度" if is_motor else f"{prefix}{i + 1}-偏航Z"

            c_x = self.plot_widget.plot(name=name_x, pen=pg.mkPen(color=color, width=2, style=Qt.SolidLine))
            c_y = self.plot_widget.plot(name=name_y, pen=pg.mkPen(color=color, width=2, style=Qt.DashLine))
            c_z = self.plot_widget.plot(name=name_z, pen=pg.mkPen(color=color, width=2, style=Qt.DotLine))
            self.curves.append((c_x, c_y, c_z))

        # 复选框滚动区
        scroll = QScrollArea()
        scroll.setFixedHeight(60)
        scroll.setWidgetResizable(True)
        cw = QWidget()
        cw.setLayout(self.chk_layout)
        scroll.setWidget(cw)
        self.layout.addWidget(scroll)

        self._controller = None

    def _on_refresh_file_list(self):
        if self._controller:
            self._controller.refresh_file_list()

    def _on_file_selected(self, item):
        """双击列表项时加载对应文件"""
        if self._controller:
            file_path = item.data(Qt.UserRole)
            self._controller.load_history_file(file_path)

    def set_controller(self, controller):
        self._controller = controller
        self.btn_focus.clicked.connect(controller.auto_focus)
        self.btn_focus_x.clicked.connect(lambda: controller.auto_focus_axis('x'))
        self.btn_focus_y.clicked.connect(lambda: controller.auto_focus_axis('y'))
        self.btn_reset.clicked.connect(controller.reset_view)
        self.btn_save.clicked.connect(controller.save_data)
        self.btn_load.clicked.connect(controller.open_history_dialog)
        # 仅当自动保存控件存在时才连接信号
        if self.chk_auto_save is not None:
            self.chk_auto_save.toggled.connect(self.spin_interval.setEnabled)
        for chk in self.checkboxes:
            chk.stateChanged.connect(self._on_checkbox_changed)

    def _on_checkbox_changed(self):
        """复选框状态改变时，调用控制器的可见性更新"""
        if self._controller:
            self._controller.update_curve_visibility()

    def _create_context_menu_wrapper(self, event):
        """右键菜单包装，转发给控制器生成菜单"""
        if self._controller:
            return self._controller.create_context_menu(event)
        return False
    def _recreate_curves_and_checkboxes(self, num_devices):
        """供控制器调用的重建接口"""
        # 清除旧曲线
        for curve_set in self.curves:
            for curve in curve_set:
                self.plot_widget.removeItem(curve)
        # 清除旧复选框
        for chk in self.checkboxes:
            chk.setParent(None)
        self.curves.clear()
        self.checkboxes.clear()
        # 清空旧布局内容
        while self.chk_layout.count():
            item = self.chk_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        # 重新创建复选框布局（需提前保存 chk_layout 为实例变量）
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
                  '#9467bd', '#8c564b', '#e377c2', '#7f7f7f']
        prefix = "电机" if self.is_motor else "传感器"

        for i in range(num_devices):
            chk = QCheckBox(f"{prefix} {i + 1}")
            chk.setChecked(True)
            color = colors[i % len(colors)]
            chk.setStyleSheet(f"color: {color}; font-weight: bold;")
            self.chk_layout.addWidget(chk)   # 需要在 __init__ 中保存为 self.chk_layout
            self.checkboxes.append(chk)

            name_x = f"{prefix}{i + 1}-位移" if self.is_motor else f"{prefix}{i + 1}-偏航X"
            name_y = f"{prefix}{i + 1}-速度" if self.is_motor else f"{prefix}{i + 1}-偏航Y"
            name_z = f"{prefix}{i + 1}-加速度" if self.is_motor else f"{prefix}{i + 1}-偏航Z"

            c_x = self.plot_widget.plot(
                name=name_x,
                pen=pg.mkPen(color=color, width=2, style=Qt.SolidLine)
            )
            c_y = self.plot_widget.plot(
                name=name_y,
                pen=pg.mkPen(color=color, width=2, style=Qt.DashLine)
            )
            c_z = self.plot_widget.plot(
                name=name_z,
                pen=pg.mkPen(color=color, width=2, style=Qt.DotLine)
            )
            self.curves.append((c_x, c_y, c_z))

        # 重新连接信号
        if self._controller:
            for chk in self.checkboxes:
                chk.stateChanged.connect(self._on_checkbox_changed)

        self.chk_layout.parentWidget().adjustSize()

class HistoryFileDialog(QDialog):
    """历史数据文件选择对话框"""
    file_selected = pyqtSignal(str)  # 发送选中的文件路径

    def __init__(self, is_motor=True, parent=None):
        super().__init__(parent)
        self.is_motor = is_motor
        self.setWindowTitle("历史数据文件 - " + ("电机" if is_motor else "传感器"))
        # 根据分辨率自动动态设置窗口大小
        screen = QApplication.primaryScreen().availableGeometry()
        self.resize(int(screen.width() * 0.4), int(screen.height() * 0.5))
        self.setMinimumSize(640, 480)   # 防止缩得过小导致控件挤爆
        self.showNormal()               # 确保不是最大化状态
        self.is_motor = is_motor
        self._init_ui()
        self.refresh_list()

    def _init_ui(self):
        layout = QVBoxLayout(self)

        # 标题标签
        title = QLabel("📁 双击文件加载历史数据")
        title.setStyleSheet("font-size: 10pt; font-weight: bold; padding: 5px;")
        layout.addWidget(title)

        # 文件列表
        self.file_list = QListWidget()
        self.file_list.setStyleSheet("""
            QListWidget {
                background-color: #F5F5F5;
                border: 1px solid #CCC;
                border-radius: 4px;
                font-size: 10pt;
            }
            QListWidget::item {
                padding: 6px;
                border-bottom: 1px solid #DDD;
            }
            QListWidget::item:selected {
                background-color: #0078D7;
                color: white;
            }
            QListWidget::item:hover {
                background-color: #E0E0E0;
            }
        """)
        self.file_list.itemDoubleClicked.connect(self._on_item_double_clicked)
        layout.addWidget(self.file_list)

        # 按钮行
        btn_layout = QHBoxLayout()
        btn_refresh = QPushButton("🔄 刷新列表")
        btn_refresh.clicked.connect(self.refresh_list)
        btn_layout.addWidget(btn_refresh)
        btn_layout.addStretch()
        btn_close = QPushButton("关闭")
        btn_close.clicked.connect(self.reject)
        btn_layout.addWidget(btn_close)
        layout.addLayout(btn_layout)

        # 状态标签
        self.status_label = QLabel("")
        self.status_label.setStyleSheet("color: #666; font-size: 10pt; padding: 3px;")
        layout.addWidget(self.status_label)

    def _get_data_dir(self):
        base = os.path.expanduser("~/.lqts/analyze_data")
        sub = "motor_data" if self.is_motor else "sensor_data"
        return os.path.join(base, sub)

    def refresh_list(self):
        self.file_list.clear()
        data_dir = self._get_data_dir()
        if not os.path.exists(data_dir):
            self.status_label.setText("数据目录不存在，请先保存数据")
            return

        files = []
        for fname in os.listdir(data_dir):
            if fname.endswith('.csv'):
                fpath = os.path.join(data_dir, fname)
                mtime = os.path.getmtime(fpath)
                size = os.path.getsize(fpath)
                files.append((fname, fpath, mtime, size))

        if not files:
            self.status_label.setText("暂无历史数据文件")
            return

        files.sort(key=lambda x: x[2], reverse=True)
        for fname, fpath, mtime, size in files:
            mtime_str = datetime.fromtimestamp(mtime).strftime("%Y-%m-%d %H:%M")
            size_str = f"{size/1024:.1f} KB" if size < 1024*1024 else f"{size/(1024*1024):.2f} MB"
            item = QListWidgetItem(f"{fname}\n{mtime_str}  {size_str}")
            item.setData(Qt.UserRole, fpath)
            item.setToolTip(f"路径: {fpath}\n大小: {size_str}\n修改时间: {mtime_str}")
            self.file_list.addItem(item)

        self.status_label.setText(f"共 {len(files)} 个文件")

    def _on_item_double_clicked(self, item):
        file_path = item.data(Qt.UserRole)
        self.file_selected.emit(file_path)

# 追加在 ui/graph_window.py 末尾

class BendGraphWindow(QDialog):
    """喷管弯曲角度历史曲线窗口（目标角度、当前角度）"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("喷管弯曲角度历史曲线")
        screen = QApplication.primaryScreen().availableGeometry()
        self.resize(int(screen.width() * 0.6), int(screen.height() * 0.7))
        self.setMinimumSize(640, 480)

        self.layout = QVBoxLayout(self)

        # 控制按钮
        btn_layout = QHBoxLayout()
        self.btn_focus = QPushButton("🎯 一键聚焦")
        self.btn_reset = QPushButton("🔄 重置视图")
        self.btn_save = QPushButton("💾 保存数据")
        btn_layout.addWidget(self.btn_focus)
        btn_layout.addWidget(self.btn_reset)
        btn_layout.addWidget(self.btn_save)
        btn_layout.addStretch()
        self.layout.addLayout(btn_layout)

        # 绘图区域
        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.PlotWidget(background='w')
        self.plot_widget.addLegend()
        self.plot_widget.setLabel('bottom', '时间', units='秒')
        self.plot_widget.setLabel('left', '角度', units='度')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.layout.addWidget(self.plot_widget)

        # 坐标显示
        self.coord_label = QLabel("点击曲线查看该点坐标")
        self.coord_label.setStyleSheet("background-color: #F5F5F5; border: 1px solid #CCC; padding: 5px;")
        self.layout.addWidget(self.coord_label)

        # 曲线对象
        self.curve_target = self.plot_widget.plot(name="目标弯曲角度", pen=pg.mkPen(color='#D13438', width=2))
        self.curve_current = self.plot_widget.plot(name="当前弯曲角度", pen=pg.mkPen(color='#107C10', width=2))

        # 数据存储
        self.time_data = []
        self.target_data = []
        self.current_data = []

        self._controller = None

    def set_controller(self, controller):
        self._controller = controller
        self.btn_focus.clicked.connect(controller.auto_focus)
        self.btn_reset.clicked.connect(controller.reset_view)
        self.btn_save.clicked.connect(controller.save_data)

    def update_data(self, times, targets, currents):
        """更新曲线数据"""
        self.time_data = times
        self.target_data = targets
        self.current_data = currents
        self.curve_target.setData(times, targets)
        self.curve_current.setData(times, currents)