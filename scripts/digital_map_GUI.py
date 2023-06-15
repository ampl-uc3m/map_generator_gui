#!/usr/bin/env python3
from tkinter import*
from tkinter import ttk,filedialog
import rospy
import numpy as np
import tf
import yaml
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped,PointStamped
from visualization_msgs.msg import Marker,MarkerArray


class WayPoint():
        
    def __init__(self, x, y, z, speed, aceleration, id_crosswalk, navigation_mode):    
        
        self.x = x
        self.y = y
        self.z = z
        self.speed = speed
        self.aceleration = aceleration
        self.id_crosswalk = id_crosswalk
        self.navigation_mode = navigation_mode
        self.ascending_number = 0
    
    def get_tuple(self):
        
        tup = (str(self.x), str(self.y), str(self.z), str(self.speed), str(self.aceleration), str(self.id_crosswalk), str(self.navigation_mode))
        return tup

class CrossWalk():
    
    def __init__(self, x, y, height, width, yaw, mode, ids):
        
        self.x = x
        self.y = y
        self.height = height
        self.width = width
        self.yaw = yaw
        self.mode = mode
        
        self.marker = Marker()
        
        self.marker.header.frame_id = "map"
        
        self.marker.ns = "threat areas"
        self.marker.type = 1                 #CUBE
        self.marker.action = 0               #ADD/MODIFY
        self.marker.id = 0
        
        while self.marker.id in ids:
            self.marker.id += 1

        self.marker_id = self.marker.id
 
        self.marker.pose.position.x = self.x 
        self.marker.pose.position.y = self.y 
        self.marker.pose.position.z = 0.0
         
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, np.deg2rad(self.yaw))
        self.marker.pose.orientation.x = quat[0]
        self.marker.pose.orientation.y = quat[1]
        self.marker.pose.orientation.z = quat[2]
        self.marker.pose.orientation.w = quat[3]
        
        self.marker.scale.x = self.width 
        self.marker.scale.y = self.height
        self.marker.scale.z = 2.0
        
        self.marker.color.r = 1
        self.marker.color.g = 0
        self.marker.color.b = 0
        self.marker.color.a = 0.5
        
        self.marker.lifetime = rospy.Duration(2.0)
        self.marker.frame_locked = True

    def get_tuple(self):
            
            tup = (str(self.x), str(self.y), str(self.height), str(self.width), str(round(self.yaw, 1)), str(self.mode))
            return tup
    
class Application():

    def __init__(self):
        
        self.waypoints_list = []
        self.crosswalk_list = []
        self.markers = MarkerArray()
        self.record = False
        self.publish_path = False
        self.publish_crosswalks = False
        
        #-----------------------------------------------------
        
        self.x = []
        self.y = []
        self.z = []
        self.Yaw = []
        self.steering = []
        self.current_steering = 0.0
        self.step = 0.1 #Min distance between waypoints
        self.counter = 0
        self.reverse = False #True to reverse path waypoints order
        
        #-----------------------------------------------------
        #Ros publishers and suscribers
        
        odom_sub = rospy.Subscriber("/ada/odometry_filtered", Odometry, self.odom_cb)
        steering_sub = rospy.Subscriber("/can/current_steering", Float64, self.steering_cb) 
        position_sub = rospy.Subscriber("/clicked_point", PointStamped, self.set_position_cb)
        
        self.pub_position = rospy.Publisher("bezier_path", Path, queue_size=1)
        self.marker_pub = rospy.Publisher("threat_areas", MarkerArray, queue_size=1)
       
        self.path_lanes = rospy.get_param("~publish_lanes", True)
        if self.path_lanes:
            self.pub_l = rospy.Publisher("left_lane", Path, queue_size=1)
            self.pub_r = rospy.Publisher("right_lane", Path, queue_size=1)
        
        #-----------------------------------------------------
        
        self.tkinter()

    def tkinter(self):
    
        #root config
        self.root = Tk()
        self.root.title("Digital map")
        self.root.geometry("700x625")
        self.root.config(bg="#323030")
        self.root.resizable(False,False)
        
        #-----------------------------------------------------------------------------------------------------------
        #Notebook config
        tabControl = ttk.Notebook(self.root)
        tabControl.pack(fill="both", expand="yes")
        
        paths_tab = Frame(tabControl, bg="#323030")
        waypoints_tab = Frame(tabControl, bg="#323030")
        crosswalk_tab = Frame(tabControl, bg="#323030")
        
        tabControl.add(paths_tab, text="Paths")
        tabControl.add(waypoints_tab, text="WayPoints")
        tabControl.add(crosswalk_tab, text="Crosswalks")
        
        #-----------------------------------------------------------------------------------------------------------
        #Path_steering_saver node GUI
        path_saver_frame = Frame(paths_tab, width=680, height=285, bg="#1e1e1e", highlightbackground="#1c1c1e", highlightthickness=1)
        path_saver_frame.place(x=10, y=10)

        path_saver_label = LabelFrame(path_saver_frame, text="Path Steering Saver", width=667, height=270, bg="#1e1e1e", fg="#ffffff")
        path_saver_label.place(x=5, y=5)
        
        start_label = Label(path_saver_label, text="Start recording path", bg="#1e1e1e", fg="#ffffff").place(x=20, y=10)
        stop_label = Label(path_saver_label, text="Stop recording path", bg="#1e1e1e", fg="#ffffff").place(x=20, y=60)
        
        start_button = Button(path_saver_label, text="Start", bg="#1e1e1e", fg="#ffffff", command=lambda:self.button_press("start")).place(x=170, y=5)
        stop_button = Button(path_saver_label, text="Stop", bg="#1e1e1e", fg="#ffffff", command=lambda:self.button_press("stop")).place(x=170, y=55)

        save_button = Button(path_saver_label, text="Save path to yaml file", bg="#1e1e1e", fg="#ffffff", command=self.path_saver).place(x=475, y=200)
        reset_button = Button(path_saver_label, text="Reset", bg="#1e1e1e", fg="#ffffff", command=lambda:self.button_press("reset")).place(x=390, y=200)

        self.recording_text = StringVar()
        self.recording_text.set("Recording OFF")
        
        self.recording_label = Label(path_saver_label, textvariable=self.recording_text, font=("Arial", 15), bg="#1e1e1e", fg="#9e3636")
        self.recording_label.place(x=510, y=5)

        self.position=StringVar()
        self.position.set("No position saved yet")
        
        self.position_label = Label(path_saver_label, textvariable=self.position, font=("Arial", 15), bg="#1e1e1e", fg="#9e3636")
        self.position_label.place(x=100, y=150)
        
        #-----------------------------------------------------------------------------------------------------------
        #pathpub node GUI
        path_publisher_frame = Frame(paths_tab, width=680, height=285, bg="#1e1e1e", highlightbackground="#1c1c1e", highlightthickness=1)
        path_publisher_frame.place(x=10, y=305)

        path_publisher_label = LabelFrame(path_publisher_frame, text= "Path Publisher", width=667, height=270, bg="#1e1e1e", fg="#ffffff")
        path_publisher_label.place(x=5, y=5)
        
        startpub_label = Label(path_publisher_label, text="Start publishing path", bg="#1e1e1e", fg="#ffffff").place(x=20, y=10)
        stoppub_label = Label(path_publisher_label, text="Stop publishing path", bg="#1e1e1e", fg="#ffffff").place(x=20, y=60)
        
        startpub_button = Button(path_publisher_label, text="Start", bg="#1e1e1e", fg="#ffffff", command=lambda:self.button_press("start_pub")).place(x=170, y=5)
        stoppub_button = Button(path_publisher_label, text="Stop", bg="#1e1e1e", fg="#ffffff", command=lambda:self.button_press("stop_pub")).place(x=170, y=55)
        
        self.pub_status = StringVar()
        self.pub_status.set("Publishing OFF")
        self.pub_status_label = Label(path_publisher_label, textvariable=self.pub_status, font=("Arial", 15), bg="#1e1e1e", fg="#9e3636")
        self.pub_status_label.place(x=510, y=5)
        
        self.file_route = StringVar()
        route_label = Label(path_publisher_label, textvariable=self.file_route, bg="#1e1e1e", fg="#27ae60").place(x=10, y=225)
        
        load_button = Button(path_publisher_label, text="Load path file", bg="#1e1e1e", fg="#ffffff", command=self.file_to_path).place(x=530, y=200)
        
        #-----------------------------------------------------------------------------------------------------------
        #Waypoints data entry
        waypoints_frame = Frame(waypoints_tab, width=680, height=580, bg="#1e1e1e", highlightbackground="#1c1c1e", highlightthickness=1)
        waypoints_frame.place(x=10, y=10)
        
        data_label = LabelFrame(waypoints_frame, text="Waypoints entry", bg="#1e1e1e", fg="#ffffff")
        data_label.place(x=10, y=5)
        
        xyz_label = Label(data_label, text="Position(RViz)", bg="#1e1e1e", fg="#ffffff").grid(column=3, row=0, columnspan=2, sticky="e", padx=(5,100), pady=10)
        
        x_label = Label(data_label, text="X:", bg="#1e1e1e", fg="#ffffff").grid(column=3, row=1, sticky="e", padx=(110,5), pady=10)
        y_label = Label(data_label, text="Y:", bg="#1e1e1e", fg="#ffffff").grid(column=3, row=2, sticky="e", padx=(110,5), pady=10)
        z_label = Label(data_label, text="Z:", bg="#1e1e1e", fg="#ffffff").grid(column=3, row=3, sticky="e", padx=(110,5), pady=10)
        
        self.x_var = DoubleVar()
        self.y_var = DoubleVar()
        self.z_var = DoubleVar()
        x_entry = Entry(data_label, textvariable= self.x_var, state="disable").grid(column=4, row=1, sticky="w", padx=(5,40), pady=10)     
        y_entry = Entry(data_label, textvariable= self.y_var, state="disable").grid(column=4, row=2, sticky="w", padx=(5,40), pady=10)     
        z_entry = Entry(data_label, textvariable= self.z_var, state="disable").grid(column=4, row=3, sticky="w", padx=(5,40), pady=10)
       
        
        speed_label = Label(data_label, text="Speed:", bg="#1e1e1e", fg="#ffffff").grid(column=0, row=0, sticky="e", padx=(40,5), pady=10)
        aceleration_label=  Label(data_label, text="Aceleration:", bg="#1e1e1e", fg="#ffffff").grid(column=0, row=1, sticky="e", padx=(40,5), pady=10)
        name_label = Label(data_label, text="ID Crosswalk:", bg="#1e1e1e", fg="#ffffff").grid(column=0, row=2, sticky="e", padx=(40,5), pady=10)
        navigation_mode_label = Label(data_label, text="Nav. mode:", bg="#1e1e1e", fg="#ffffff").grid(column=0, row=3, sticky="e", padx=(40,5), pady=10)

        self.speed_var = DoubleVar()
        self.aceleration_var = DoubleVar()
        self.id_crosswalk_var = IntVar()
        self.nav_mode_var = IntVar()
        speed_entry = Entry(data_label, textvariable= self.speed_var).grid(column=1, row=0, sticky="w")
        aceleration_entry = Entry(data_label, textvariable= self.aceleration_var).grid(column=1, row=1, sticky="w")
        id_crosswalk_entry = ttk.Combobox(data_label, textvariable= self.id_crosswalk_var, values= ["0", "-1"]).grid(column=1, row=2, sticky="w")
        navigation_mode_entry = ttk.Combobox(data_label, textvariable= self.nav_mode_var, values= ["0", "1"]).grid(column=1, row=3, sticky="w")
        
        create_button = Button(data_label, text="Create", width=4, bg="#1e1e1e", fg="#ffffff", command=lambda:self.button_press("create_wp"))
        clear_button = Button(data_label, text="Clear", width=4, bg="#1e1e1e", fg="#ffffff", command=lambda:self.button_press("clear_data_wp"))
        create_button.grid(column=4, row=5, sticky="e", padx=(0,90), pady=10)
        clear_button.grid(column=4, row=5, sticky="e", padx=(0,10), pady=10)
        
        #-----------------------------------------------------------------------------------------------------------
        #waypoints list display       
        columns = ("x", "y", "z", "Speed", "Aceleration", "ID Crosswalk", "Navigation Mode")
        
        self.tree = ttk.Treeview(waypoints_frame, selectmode="browse", columns=columns, show="headings")
        self.tree.place(x=10, y=265, width=645, height=260)
    
        #Define columns format
        self.tree.column("x", anchor=W, width=120)
        self.tree.column("y", anchor=W, width=120)
        self.tree.column("z", anchor=W, width=120)
        self.tree.column("Speed", anchor=W, width=120)
        self.tree.column("Aceleration", anchor=W, width=120)
        self.tree.column("ID Crosswalk", anchor=CENTER, width=120)
        self.tree.column("Navigation Mode", anchor=CENTER, width=120)
        
        #Define headings format
        self.tree.heading("x", text="X", anchor=CENTER)
        self.tree.heading("y", text="Y", anchor=CENTER)
        self.tree.heading("z", text="Z", anchor=CENTER)
        self.tree.heading("Speed", text="Speed", anchor=CENTER)
        self.tree.heading("Aceleration", text="Aceleration", anchor=CENTER)
        self.tree.heading("ID Crosswalk", text="ID Crosswalk", anchor=CENTER)
        self.tree.heading("Navigation Mode", text="Navigation Mode", anchor=CENTER)
        
        #Creating srollbars
        vs = ttk.Scrollbar(waypoints_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=vs.set)
        vs.place(x=655, y=265, height=260)
        
        hs = ttk.Scrollbar(waypoints_frame, orient="horizontal", command=self.tree.xview)
        self.tree.configure(xscrollcommand=hs.set)
        hs.place(x=10, y=510, width=645)
        
        #Control Buttons
        save_wp = Button(waypoints_frame, text="Generate \".yaml\"", bg="#1e1e1e", fg="#ffffff", command=self.save_yaml)
        load_wp = Button(waypoints_frame, text="Load \".yaml\"", bg="#1e1e1e", fg="#ffffff", command=self.load_yaml)
        delete_wp = Button(waypoints_frame, text="Delete waypoint", bg="#1e1e1e", fg="#ffffff", command=self.delete_waypoint)
        save_wp.place(x=405, y=535)
        load_wp.place(x=557, y=535)
        delete_wp.place(x=10, y=535)
        
        #-----------------------------------------------------------------------------------------------------------
        #Crosswalks data entry
        crosswalks_frame = Frame(crosswalk_tab, width=680, height=580, bg="#1e1e1e", highlightbackground="#1c1c1e", highlightthickness=1)
        crosswalks_frame.place(x=10, y=10)
        
        data_label_cw = LabelFrame(crosswalks_frame, text="Crosswalk entry", bg="#1e1e1e", fg="#ffffff")
        data_label_cw.place(x=10, y=5)
        
        xy_label_cw = Label(data_label_cw, text="Position(RViz)", bg="#1e1e1e", fg="#ffffff").grid(column=3, row=0, columnspan=2, sticky="e", padx=(5,175), pady=10)
        
        x_label_cw = Label(data_label_cw, text="X:", bg="#1e1e1e", fg="#ffffff").grid(column=3, row=1, sticky="e", padx=(60,5), pady=10)
        y_label_cw = Label(data_label_cw, text="Y:", bg="#1e1e1e", fg="#ffffff").grid(column=3, row=2, sticky="e", padx=(60,5), pady=10)

        self.x_var_cw = DoubleVar()
        self.y_var_cw = DoubleVar()
        x_entry_cw = Entry(data_label_cw, textvariable= self.x_var_cw).grid(column=4, row=1, sticky="w", padx=(5,78), pady=10)     
        y_entry_cw = Entry(data_label_cw, textvariable= self.y_var_cw).grid(column=4, row=2, sticky="w", padx=(5,78), pady=10)     

        height_label = Label(data_label_cw, text="Height:", bg="#1e1e1e", fg="#ffffff").grid(column=0, row=0, sticky="e", padx=(40,5), pady=10)
        width_label = Label(data_label_cw, text="Width:", bg="#1e1e1e", fg="#ffffff").grid(column=0, row=1, sticky="e", padx=(40,5), pady=10)
        yaw_lebel = Label(data_label_cw, text="Yaw (Degrees):", bg="#1e1e1e", fg="#ffffff").grid(column=0, row=2, sticky="e", padx=(40,5), pady=10)
        mode_label = Label(data_label_cw, text="Mode:", bg="#1e1e1e", fg="#ffffff").grid(column=0, row=3, sticky="e", padx=(40,5), pady=10)
        
        self.height_var = DoubleVar()
        self.width_var = DoubleVar()
        self.yaw_var = DoubleVar()
        self.mode_var = DoubleVar()
        height_entry = Entry(data_label_cw, textvariable= self.height_var).grid(column=1, row=0, sticky="w")
        width_entry = Entry(data_label_cw, textvariable= self.width_var).grid(column=1, row=1, sticky="w")
        yaw_entry = Entry(data_label_cw, textvariable= self.yaw_var).grid(column=1, row=2, sticky="w")
        mode_entry = ttk.Combobox(data_label_cw, textvariable= self.mode_var, values= ["0.0", "1.0"]).grid(column=1, row=3, sticky="w")
        
        create_button_cw = Button(data_label_cw, text="Create", width=4, bg="#1e1e1e", fg="#ffffff", command=lambda:self.button_press("create_cw"))
        clear_button_cw = Button(data_label_cw, text="Clear", width=4, bg="#1e1e1e", fg="#ffffff", command=lambda:self.button_press("clear_data_cw"))
        create_button_cw.grid(column=4, row=3, sticky="e", padx=(0,90), pady=10)
        clear_button_cw.grid(column=4, row=3, sticky="e", padx=(0,10), pady=10)
        
        #-----------------------------------------------------------------------------------------------------------
        #crosswalk list display       
        columns2 = ("x", "y", "Height", "Width", "Yaw", "Mode")
        
        self.tree2 = ttk.Treeview(crosswalks_frame, selectmode="browse", columns=columns2, show="headings")
        self.tree2.place(x=10, y=220, width=645, height=230)
    
        #Define columns format
        self.tree2.column("x", anchor=W, width=120)
        self.tree2.column("y", anchor=W, width=120)
        self.tree2.column("Height", anchor=W, width=120)
        self.tree2.column("Width", anchor=W, width=120)
        self.tree2.column("Yaw", anchor=W, width=120)
        self.tree2.column("Mode", anchor=CENTER, width=120)
        
        #Define headings format
        self.tree2.heading("x", text="X", anchor=CENTER)
        self.tree2.heading("y", text="Y", anchor=CENTER)
        self.tree2.heading("Height", text="Height", anchor=CENTER)
        self.tree2.heading("Width", text="Width", anchor=CENTER)
        self.tree2.heading("Yaw", text="Yaw", anchor=CENTER)
        self.tree2.heading("Mode", text="Mode", anchor=CENTER)
        
        #Creating srollbars
        vs2 = ttk.Scrollbar(crosswalks_frame, orient="vertical", command=self.tree2.yview)
        self.tree2.configure(yscrollcommand=vs2.set)
        vs2.place(x=655, y=220, height=230)
        
        hs2 = ttk.Scrollbar(crosswalks_frame, orient="horizontal", command=self.tree2.xview)
        self.tree2.configure(xscrollcommand=hs2.set)
        hs2.place(x=10, y=435, width=645)
        
        #Control Buttons
        save_yaml = Button(crosswalks_frame, text="Generate \".yaml\"", bg="#1e1e1e", fg="#ffffff", command=self.save_yaml)
        load_yaml = Button(crosswalks_frame, text="Load \".yaml\"", bg="#1e1e1e", fg="#ffffff", command=self.load_yaml)
        delete_cw = Button(crosswalks_frame, text="Delete waypoint", bg="#1e1e1e", fg="#ffffff", command=self.delete_crosswalk)
        save_yaml.place(x=405, y=460)
        load_yaml.place(x=557, y=460)
        delete_cw.place(x=10, y=460)   
        
        #-----------------------------------------------------------------------------------------------------------
        #Crosswalk publisher
        pub_crosswalk_label = LabelFrame(crosswalks_frame, text="Crosswalk publisher", width=660, height=70, bg="#1e1e1e", fg="#ffffff")
        pub_crosswalk_label.place(x=10, y=500)
        
        start_pub_cw = Button(pub_crosswalk_label, text="Publish crosswalks", bg="#1e1e1e", fg="#ffffff", command=lambda:self.button_press("start_pub_cw"))
        stop_pub_cw = Button(pub_crosswalk_label, text="Stop publishing crosswalks", bg="#1e1e1e", fg="#ffffff", command=lambda:self.button_press("stop_pub_cw"))
        start_pub_cw.place(x=20, y=10)
        stop_pub_cw.place(x=190, y=10)
        
        self.pub_cw_status = StringVar()
        self.pub_cw_status.set("OFF")
        self.pub_cw_status_label = Label(pub_crosswalk_label, textvariable=self.pub_cw_status, font=("Arial", 15), bg="#1e1e1e", fg="#9e3636")
        self.pub_cw_status_label.place(x=520, y=10)

    def button_press(self, msg):
        
        if msg == "start":
            self.recording_text.set("Recording ON")
            self.recording_label.config(fg="#27ae60")
            self.record = True
        elif msg == "stop":
            self.recording_text.set("Recording OFF")
            self.recording_label.config(fg="#9e3636")
            self.record = False
        elif msg == "reset":
            self.x.clear()
            self.y.clear()
            self.z.clear()
            self.Yaw.clear()
            self.steering.clear()
            self.current_steering = 0.0
            self.counter = 0
            self.position_label.config(fg="#9e3636")
            self.position.set("No position saved yet")
        elif msg == "start_pub":
            self.publish_path = True
            self.pub_status_label.config(fg="#27ae60")
            self.pub_status.set("Publishing ON")
            self.publish_method()
        elif msg == "stop_pub":
            self.publish_path = False
            self.pub_status_label.config(fg="#9e3636")
            self.pub_status.set("Publishing OFF")
        elif msg == "clear_data_wp":
            self.x_var.set("")     
            self.y_var.set("")      
            self.z_var.set("")  
            self.speed_var.set("") 
            self.aceleration_var.set("")
            self.id_crosswalk_var.set(0)
            self.nav_mode_var.set(0) 
        elif msg == "create_wp":
            self.add_waypoint()
        elif msg == "clear_data_cw":
            self.x_var_cw.set("")     
            self.y_var_cw.set("")      
            self.height_var.set("")
            self.width_var.set("")
            self.yaw_var.set("")
            self.mode_var.set(0.0)
        elif msg == "create_cw":
            self.add_crosswalk()
        elif msg == "start_pub_cw":
            self.publish_crosswalks = True
            self.pub_cw_status_label.config(fg="#27ae60")
            self.pub_cw_status.set("ON")
            self.publish_method()
        elif msg == "stop_pub_cw":
            self.publish_crosswalks = False
            self.pub_cw_status_label.config(fg="#9e3636")
            self.pub_cw_status.set("OFF")

    def file_to_path(self):
        
        path_to_file = filedialog.askopenfilename(title="Open path file", defaultextension=".yaml", filetypes=[("yaml file", "*.yaml")])
        self.file_route.set(path_to_file)
        
        f = open(path_to_file, "r")

        fl =f.read().splitlines()

        path = []

        for x in fl:
            if x[:2] == "[[":
                vec = []
                t = x[2:len(x)-2].split(", ")
                for n in t:
                    vec.append(float(n))
                path.append(vec)
                #vec.append(x[1:len(x)-1])
            else:
                vec = []
                t = x[1:len(x)-2].split(", ")
                for n in t:
                    vec.append(float(n))
                path.append(vec)

        i=0
        frame_id = rospy.get_param("~frame", "map")

        self.path_msg = Path()
        self.l_path = Path()
        self.r_path = Path()
        
        self.path_msg.header.frame_id = frame_id
        self.l_path.header.frame_id = frame_id
        self.r_path.header.frame_id = frame_id
        X = []
        Y = []
        Z = []
        Roll = []
        Pitch = []
        Yaw = []
        l_lane = []
        r_lane = []
        lane_size = 6.5
        last_x = 0.0
        last_y = 0.0
        min_dist = 0.2
        x = 0
        y = 0

        for i in range(len(path)):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id
            pose_stamped.pose.position.x = path[i][0]
            pose_stamped.pose.position.y = path[i][1]
            pose_stamped.pose.position.z = 0

            yaw = 0.0
            if i == (len(path) - 1):
                yaw = np.arctan2((path[i][1] - path[i-1][1]), (path[i][0] - path[i-1][0]))
            else:
                yaw = np.arctan2((path[i+1][1] - path[i][1]), (path[i+1][0] - path[i][0]))

            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
            # print(quat)
            pose_stamped.pose.orientation.x = quat[0]
            pose_stamped.pose.orientation.y = quat[1]
            pose_stamped.pose.orientation.z = quat[2]
            pose_stamped.pose.orientation.w = quat[3]
            self.path_msg.poses.append(pose_stamped)

            if self.path_lanes:
                x = path[i][0]
                y = path[i][1]

                if(np.sqrt((last_x - x)**2 + (last_y - y)**2) < min_dist):
                    continue
                last_x = x
                last_y = y

                X.append(x)
                Y.append(y)
                Z.append(0)
                # roll, pitch, yaw = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                Roll.append(0)
                Pitch.append(0)
                Yaw.append(yaw)
                l_point_x = X[-1] + lane_size/2.0 * np.cos(yaw + np.pi/2.0)
                l_point_y = Y[-1] + lane_size/2.0 * np.sin(yaw + np.pi/2.0)
                l_lane.append([l_point_x, l_point_y, Z[-1]])

                r_point_x = X[-1] + lane_size/2.0 * np.cos(yaw - np.pi/2.0)
                r_point_y = Y[-1] + lane_size/2.0 * np.sin(yaw - np.pi/2.0)
                r_lane.append([r_point_x, r_point_y, Z[-1]])

                # quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

                l_pose = PoseStamped()
                # l_pose.header.frame_id = "map"
                l_pose.pose.position.x = l_point_x
                l_pose.pose.position.y = l_point_y
                l_pose.pose.position.z = Z[-1]
                l_pose.pose.orientation.x = quat[0]
                l_pose.pose.orientation.y = quat[1]
                l_pose.pose.orientation.z = quat[2]
                l_pose.pose.orientation.w = quat[3]
                self.l_path.poses.append(l_pose)

                r_pose = PoseStamped()
                # r_pose.header.frame_id = "map"
                r_pose.pose.position.x = r_point_x
                r_pose.pose.position.y = r_point_y
                r_pose.pose.position.z = Z[-1]
                r_pose.pose.orientation.x = quat[0]
                r_pose.pose.orientation.y = quat[1]
                r_pose.pose.orientation.z = quat[2]
                r_pose.pose.orientation.w = quat[3]
                self.r_path.poses.append(r_pose)

    def publish_method(self):
        
        if self.publish_crosswalks:
            self.marker_pub.publish(self.markers)
        if self.publish_path:
            self.pub_position.publish(self.path_msg)
            if self.path_lanes:
                self.pub_l.publish(self.l_path)
                self.pub_r.publish(self.r_path)
        if self.publish_path or self.publish_crosswalks:
            self.root.after(1000, self.publish_method)

    def path_saver(self):
        
        self.record = False
        
        if self.reverse:
            self.x.reverse()
            self.y.reverse()
            self.z.reverse()

        file = filedialog.asksaveasfilename(title="Saving data to yaml file", defaultextension=".yaml", filetypes=[("yaml file", "*.yaml")])
        
        raw_file = open(file, "w+")

        # Write path file headers
        raw_file.write("[")

        for i in range(len(self.x)):
            # Generate raw path file
            raw_file.write("[%s " % (self.x[i]))
            raw_file.write(", %s " % (self.y[i]))
            raw_file.write(", %s " % (self.Yaw[i]))

            if i < (len(self.x) - 1):
                raw_file.write(", %s],\n" % self.steering[i])
            else:
                # raw_file.write(", %s]" % steering)
                raw_file.write(", %s]" % self.steering[i])
            
        raw_file.write("]")

        # Close all file descriptors
        raw_file.close()

    def add_waypoint(self):
        
        x = self.x_var.get()
        y = self.y_var.get()
        z = self.z_var.get()
        speed = self.speed_var.get()
        aceleration = self.aceleration_var.get()
        id_crosswalk = self.id_crosswalk_var.get()
        navigation_mode = self.nav_mode_var.get()

        self.waypoints_list.append(WayPoint(x, y, z, speed, aceleration, id_crosswalk, navigation_mode))
        
        for i in range(len(self.waypoints_list)):
                self.waypoints_list[i].ascending_number = i*5
                        
        values = (x, y, z, speed, aceleration, id_crosswalk, navigation_mode)
        self.tree.insert(parent='',index="end", values=values)

    def add_crosswalk(self):

        x = self.x_var_cw.get()
        y = self.y_var_cw.get()
        height = self.height_var.get()
        width = self.width_var.get()
        yaw = self.yaw_var.get()
        mode = self.mode_var.get()
        
        ids = []
        for i in self.crosswalk_list:
            ids.append(i.marker.id)
            
        self.crosswalk_list.append(CrossWalk(x, y, height, width, yaw, mode, ids))
        self.markers.markers.append(self.crosswalk_list[-1].marker)
        
        values = (x, y, height, width, round(yaw, 1), mode)
        self.tree2.insert(parent='',index="end", values=values)
         
    def delete_waypoint(self):
        
        selected = self.tree.focus()
        aux_tuple = self.tree.item(selected, 'values')
        tree = self.tree.delete(selected)
        
        for i in self.waypoints_list: 
            if i.get_tuple() == aux_tuple:
                self.waypoints_list.remove(i)
                break
        
        for i in range(len(self.waypoints_list)):
            self.waypoints_list[i].ascending_number = i*5
            
    def delete_crosswalk(self):
    
        selected = self.tree2.focus()
        aux_tuple = self.tree2.item(selected, 'values')
        tree = self.tree2.delete(selected)
        
        for i in self.crosswalk_list: 

            if i.get_tuple() == aux_tuple:
                self.markers.markers.remove(i.marker)
                self.crosswalk_list.remove(i)
                break

    def save_yaml(self):
        
        file_rute = filedialog.asksaveasfilename(title="Saving data to yaml file", defaultextension=".yaml", filetypes=[("yaml file", "*.yaml")])
        file = open(file_rute, "w+")  
           
        file.write("speed_limit_profile: [\n")
        for i in range(len(self.waypoints_list)):
            file.write(" [{}".format(self.waypoints_list[i].x))
            file.write(", {}".format(self.waypoints_list[i].y))
            file.write(", {}".format(self.waypoints_list[i].z)) 
            file.write(", {}".format(self.waypoints_list[i].speed)) 
            file.write(", {}".format(self.waypoints_list[i].aceleration))  
            file.write(", {}".format(self.waypoints_list[i].id_crosswalk)) 
            file.write(", {}".format(self.waypoints_list[i].navigation_mode))
            
            if i < (len(self.waypoints_list) - 1):
                file.write(", {}],\n".format(self.waypoints_list[i].ascending_number))
            else:
                file.write(", {}]\n".format(self.waypoints_list[i].ascending_number))
        file.write("]\n\n")
        
        file.write("/ada/threat_areas: [\n")
        for i in range(len(self.crosswalk_list)):
            file.write(" [{}".format(self.crosswalk_list[i].x))
            file.write(", {}".format(self.crosswalk_list[i].y))
            file.write(", {}".format(self.crosswalk_list[i].width)) 
            file.write(", {}".format(self.crosswalk_list[i].height)) 
            file.write(", {}".format(round(np.deg2rad(self.crosswalk_list[i].yaw),2)))  

            if i < (len(self.crosswalk_list) - 1):
                file.write(", {}],\n".format(self.crosswalk_list[i].mode))
            else:
                file.write(", {}]\n".format(self.crosswalk_list[i].mode))
        file.write("]\n\n")
        
        file.close()

    def load_yaml(self):
        
        file_rute = filedialog.askopenfilename(title="Open path file", defaultextension=".yaml", filetypes=[("yaml file", "*.yaml")])
        
        with open(file_rute, "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)

        wp = data["speed_limit_profile"]
        cw = data["/ada/threat_areas"]
        
        self.waypoints_list.clear()
        self.crosswalk_list.clear()
        self.markers.markers.clear()
        ids = []
        
        for i in range(len(wp)):
                x = wp[i][0]
                y = wp[i][1]
                z = wp[i][2]
                speed = wp[i][3]
                aceleration = wp[i][4]
                id_crosswalk = wp[i][5]
                mode = wp[i][6]
                self.waypoints_list.append(WayPoint(x, y, z, speed, aceleration, id_crosswalk, mode))
        
        for i in range(len(self.waypoints_list)):
            self.waypoints_list[i].ascending_number = i*5        
        
        for i in range(len(cw)):
                x = cw[i][0]
                y = cw[i][1]
                width = cw[i][2]
                height= cw[i][3]
                yaw = np.rad2deg(cw[i][4])
                mode = cw[i][5]
                self.crosswalk_list.append(CrossWalk(x, y, height, width, yaw, mode, ids))
                ids.append(i)
                self.markers.markers.append(self.crosswalk_list[i].marker)
        
        self.tree.delete(*self.tree.get_children())
        self.tree2.delete(*self.tree2.get_children())
        
        for i in self.waypoints_list:
            self.tree.insert(parent='',index="end", values=i.get_tuple())
        
        for i in self.crosswalk_list:
            self.tree2.insert(parent='',index="end", values=i.get_tuple())

    def odom_cb(self, message):
        
        if self.record == True:
            if len(self.x) == 0:
                self.x.append(message.pose.pose.position.x)
                self.y.append(message.pose.pose.position.y)
                self.z.append(message.pose.pose.position.z)
                _, _, yaw = tf.transformations.euler_from_quaternion([message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w])
                self.Yaw.append(yaw)
                self.steering.append(self.current_steering)
                print("Position ", self.counter, " saved!")
                self.position_label.config(fg="#27ae60")
                self.position.set("Position " + str(self.counter) + " saved!")
                self.counter += 1
                return
            dist = np.sqrt((self.x[-1] - message.pose.pose.position.x)**2 + (self.y[-1] - message.pose.pose.position.y)**2)
            if dist > self.step:
                self.x.append(message.pose.pose.position.x)
                self.y.append(message.pose.pose.position.y)
                self.z.append(message.pose.pose.position.z)
                _, _, yaw = tf.transformations.euler_from_quaternion([message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w])
                self.Yaw.append(yaw)
                self.steering.append(self.current_steering)
                print("Position ", self.counter, " saved!")
                self.position_label.config(fg="#27ae60")
                self.position.set("Position " + str(self.counter) + " saved!")
                self.counter += 1

    def steering_cb(self, message):
        
        self.current_steering = message.data                

    def set_position_cb(self, message):
        self.x_var.set(round(message.point.x,2))
        self.y_var.set(round(message.point.y,2))
        self.z_var.set(round(message.point.z,2))
        
        self.x_var_cw.set(round(message.point.x,2))
        self.y_var_cw.set(round(message.point.y,2))

if __name__ == "__main__":

    rospy.init_node("digital_map_GUI")
    rospy.loginfo("digital_map_GUI node has been started") 
    app = Application()

    app.root.mainloop()