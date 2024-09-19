import sys
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QLabel, QFileDialog, QPushButton,
    QSlider, QMessageBox
)
import open3d as o3d
import numpy as np
from scipy.spatial import Delaunay
from functools import reduce
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D

class PointCloudVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Volume Calculation of a Point Cloud")
        self.showMaximized()
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout()
        self.central_widget.setLayout(self.main_layout)

        # Create widget for buttons and sliders
        self.control_widget = QWidget()
        self.control_layout = QVBoxLayout()
        self.control_widget.setLayout(self.control_layout)
        self.control_widget.setStyleSheet("background-color: #e6e6e6;")

        # Left side layout for sliders and buttons
        self.left_layout = QHBoxLayout()
        self.control_layout.addLayout(self.left_layout)

        # Right side layout for plots
        self.right_layout = QVBoxLayout()
        self.control_layout.addLayout(self.right_layout)

        # Layout for buttons
        self.button_layout = QVBoxLayout()

        self.create_points_layout = QVBoxLayout()
        self.create_points_layout.addWidget(QLabel("Filter point cloud"))
        self.number_points_slider = self.create_slider("Number of Points", 0, 500, 500)
        self.height_slider = self.create_slider("Height", 0, 50, 25)
        self.xy_min_slider = self.create_slider("XY minimum", -100, 100, 25)
        self.xy_max_slider = self.create_slider("XY maximum", -100, 100, 25)
        self.left_layout.addLayout(self.create_points_layout)

        self.filter_layout = QVBoxLayout()
        self.z_max_slider = self.create_slider("Z Max", -100, 100, 100)
        self.z_min_slider = self.create_slider("Z Min", -100, 100, 0)
        self.y_max_slider = self.create_slider("Y Max", -100, 100, 100)
        self.y_min_slider = self.create_slider("Y Min", -100, 100, -100)
        self.x_max_slider = self.create_slider("X Max", -100, 100, 100)
        self.x_min_slider = self.create_slider("X Min", -100, 100, -100)
        self.left_layout.addLayout(self.filter_layout)

        # Add buttons and sliders to the left layout
        self.point_cloud_file_label = QLabel("")
        self.point_cloud_file_path = ""
        self.point_cloud_file_button = QPushButton("Choose File")
        self.point_cloud_file_button.clicked.connect(self.choose_point_cloud_file)
        self.button_layout.addWidget(self.point_cloud_file_label)
        self.button_layout.addWidget(self.point_cloud_file_button)

        self.visualize_button = QPushButton("Visualize")
        self.visualize_button.clicked.connect(self.visualize_point_cloud)
        self.button_layout.addWidget(self.visualize_button)

        self.calculate_volume_delaunay_button = QPushButton("Calculate Volume and Area")
        self.calculate_volume_delaunay_button.clicked.connect(self.calculate_volume_delaunay)
        self.button_layout.addWidget(self.calculate_volume_delaunay_button)

        self.save_visualization_button = QPushButton("Save Visualization")
        self.save_visualization_button.clicked.connect(self.save_visualization)
        self.button_layout.addWidget(self.save_visualization_button)

        self.export_results_button = QPushButton("Export Results")
        self.export_results_button.clicked.connect(self.export_results)
        self.button_layout.addWidget(self.export_results_button)

        self.left_layout.addLayout(self.button_layout)

        # Add the control widget to the main layout
        self.main_layout.addWidget(self.control_widget)

        # Placeholder for point cloud visualization widget
        self.point_cloud_widget = QWidget()
        self.point_cloud_layout = QHBoxLayout()
        self.point_cloud_widget.setLayout(self.point_cloud_layout)
        self.main_layout.addWidget(self.point_cloud_widget)

        # Placeholder for footer widget
        self.footer_widget = QWidget()
        self.footer_layout = QHBoxLayout()
        self.footer_widget.setStyleSheet("background-color: #8699a1")
        self.footer_layout.addWidget(QLabel("Created by Jorge Silva DEM Universidade de Aveiro 2024"))
        self.footer_widget.setLayout(self.footer_layout)
        self.main_layout.addWidget(self.footer_widget)

        # Adjusting the stretch factor to ensure proper resizing
        self.main_layout.setStretchFactor(self.control_widget, 0)
        self.main_layout.setStretchFactor(self.point_cloud_widget, 1)
        self.main_layout.setStretchFactor(self.footer_widget, 0)

    def create_slider(self, label, min_val, max_val, default_val):
        slider_layout = QHBoxLayout()
        label_widget = QLabel(label)
        slider = QSlider()
        slider.setOrientation(1)  # Vertical orientation
        slider.setMinimum(min_val * 1000)  # Adjusted for increments of 0.1
        slider.setMaximum(max_val * 1000)  # Adjusted for increments of 0.1
        slider.setValue(default_val * 1000)  # Adjusted for increments of 0.1
        slider.setSingleStep(1)  # Set single step to 1
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval((max_val - min_val) // 1000)
        slider_label = QLabel(str(default_val))  # Display the initial value
        slider.valueChanged.connect(lambda val, label=slider_label: label.setText(str(val / 1000)))
        slider_layout.addWidget(label_widget)
        slider_layout.addWidget(slider)
        slider_layout.addWidget(slider_label)
        self.create_points_layout.addLayout(slider_layout)
        return slider



    def choose_point_cloud_file(self):
        file_dialog = QFileDialog()
        file_dialog.setNameFilter("Point Cloud Files (*.pcd)")
        if file_dialog.exec_():
            self.point_cloud_file_path = file_dialog.selectedFiles()[0]
            self.point_cloud_file_label.setText(self.point_cloud_file_path)

    def visualize_point_cloud(self):
        if not self.point_cloud_file_path:
            QMessageBox.warning(self, "Warning", "Please choose a point cloud file.")
            return None

        # Clear existing plot, if any
        for i in reversed(range(self.point_cloud_layout.count())):
            widget = self.point_cloud_layout.itemAt(i).widget()
            if widget is not None:
                widget.deleteLater()

        pcd = o3d.io.read_point_cloud(self.point_cloud_file_path)
        print("Point cloud loaded:", pcd)

        if pcd.is_empty():
            QMessageBox.warning(self, "Warning", "The point cloud file is empty or could not be loaded.")
            return None

        number_points = self.number_points_slider.value()
        p1_load = np.asarray(pcd.points)
        rand_xy = np.random.uniform(self.xy_min_slider.value() / 1000, self.xy_max_slider.value() / 1000,
                                    (number_points, 2))
        rand_z = np.full((1, number_points), self.height_slider.value() / 1000)
        p1 = np.concatenate((rand_xy, rand_z.T), axis=1)
        pointSet = o3d.geometry.PointCloud()
        pointSet.points = o3d.utility.Vector3dVector(p1)

        p3_load = np.concatenate((p1_load, p1), axis=0)
        pcd1 = o3d.geometry.PointCloud()
        pcd1.points = o3d.utility.Vector3dVector(p3_load)

        # Find the minimum Z value
        min_z = np.min(np.asarray(pcd1.points)[:, 2])

        # Translate points so that the lowest point is at Z = 0
        translation = np.array([0, 0, -min_z])
        pcd1.translate(translation)

        x_min, x_max = self.x_min_slider.value() / 1000, self.x_max_slider.value() / 1000
        y_min, y_max = self.y_min_slider.value() / 1000, self.y_max_slider.value() / 1000
        z_min, z_max = self.z_min_slider.value() / 1000, self.z_max_slider.value() / 1000

        voxel_size = 0.05
        pcd1_downsampled = pcd1.voxel_down_sample(voxel_size)

        passthrough = o3d.geometry.PointCloud()
        passthrough.points = o3d.utility.Vector3dVector([[x, y, z] for [x, y, z] in pcd1_downsampled.points if
                                                         x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max])

        points = np.asarray(passthrough.points)
        print("Filtered points:", points)

        if points.size == 0:
            QMessageBox.warning(self, "Warning", "No points remain after filtering.")
            return None

        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111, projection='3d')

        self.ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Point Cloud Visualization')

        canvas = FigureCanvas(self.figure)
        self.point_cloud_layout.addWidget(canvas)
        self.latest_fig = self.figure

        return points

    def calculate_volume_delaunay(self):
        # Show a "Waiting" message
        waiting_message = QMessageBox(self)
        waiting_message.setWindowTitle("Processing")
        waiting_message.setText("Please wait while the Delaunay triangulation is being processed...")
        waiting_message.show()
        QApplication.processEvents()  # Ensure the message is displayed immediately

        for i in reversed(range(self.point_cloud_layout.count())):
            widget = self.point_cloud_layout.itemAt(i).widget()
            if widget is not None:
                widget.deleteLater()
        extract_xy = []
        points = self.visualize_point_cloud()
        for point in points:
            extract_xy.append([point[0], point[1]])
        index = Delaunay(np.array(extract_xy))

        # Update visualization to show Delaunay triangulation
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        def get_triangles_vertices(triangles, vertices):
            triangles_vertices = []
            for triangle in triangles:
                new_triangles_vertices = [vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]]
                triangles_vertices.append(new_triangles_vertices)
            return np.array(triangles_vertices)

        def volume_triangular_prism(triangle):
            p1, p2, p3 = triangle
            x1, y1, z1 = p1
            x2, y2, z2 = p2
            x3, y3, z3 = p3
            area = (1 / 2) * (x1 * y2 - x2 * y1 + x2 * y3 - x3 * y2 + x3 * y1 - x1 * y3)
            height = (1 / 3) * (z1 + z2 + z3)
            volume = area * height
            return area, volume

        # Plot the triangles
        triangles_vertices = get_triangles_vertices(index.simplices, points)
        for triangle in triangles_vertices:
            x = [point[0] for point in triangle]
            y = [point[1] for point in triangle]
            z = [point[2] for point in triangle]
            ax.plot(x + [x[0]], y + [y[0]], z + [z[0]], color='red',
                    linewidth=0.2)  # Connect the last point to the first to close the triangle

        total_area = 0
        total_volume = 0
        for triangle in triangles_vertices:
            area, volume = volume_triangular_prism(triangle)
            total_area += area
            total_volume += volume

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        canvas = FigureCanvas(fig)
        self.point_cloud_layout.addWidget(canvas)

        # Close the "Waiting" message
        waiting_message.close()

        QMessageBox.information(self, "Volume and Area Calculation",
                                f"The volume is {round(total_volume, 3)} m³\n"
                                f"The surface area is {round(total_area, 3)} m²")

        return total_volume, total_area

    def save_visualization(self):
        file_dialog = QFileDialog()
        file_path, _ = file_dialog.getSaveFileName(self, "Save File", "", "PNG (*.png);;JPEG (*.jpg *.jpeg)")
        if file_path:
            self.latest_fig.savefig(file_path)

    def export_results(self):
        file_dialog = QFileDialog()
        file_path, _ = file_dialog.getSaveFileName(self, "Save File", "", "Text Files (*.txt)")
        if file_path:
            with open(file_path, 'w') as file:
                file.write(f"Volume: {round(self.calculated_volume, 3)} m³\n")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = PointCloudVisualizer()
    window.show()
    sys.exit(app.exec_())

