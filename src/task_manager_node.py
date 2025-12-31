#!/usr/bin/env python3
import actionlib
import os
import time
from typing import Dict, List, Any
import rospy
import yaml
import tf
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger
from std_msgs.msg import String

class TaskManager:
    def __init__(self):
        mission_path = rospy.get_param("~mission", "")
        if not mission_path:
            mission_path = "/home/nur/catkin_ws/src/robot_cleaner/config/mission.yaml"
            rospy.logwarn(f"Mission path eksik, varsayılan: {mission_path}")

        self._mission = self._load_mission(mission_path)
        self._report_path = rospy.get_param("~report_path", os.path.expanduser("~/.ros/clean_report.txt"))
        self._goal_timeout = rospy.get_param("~goal_timeout", 60.0)
        self._room_timeout = rospy.get_param("~room_timeout", 120.0)
        self._max_retries = int(rospy.get_param("~max_retries", 1))

        self._qr_service_name = "/qr_reader_node/scan"
        rospy.wait_for_service(self._qr_service_name)
        self._qr_client = rospy.ServiceProxy(self._qr_service_name, Trigger)

        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._goal_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self._goal_client.wait_for_server()
        self._report_pub = rospy.Publisher("task_report", String, queue_size=10, latch=True)
        self._reports: List[str] = [] 

        rospy.loginfo("GÖREVE HAZIR.")

    @staticmethod
    def _load_mission(path: str) -> Dict[str, Any]:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    @staticmethod
    def _pose_from_dict(pose_dict: Dict[str, float]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = pose_dict.get("x", 0.0)
        pose.pose.position.y = pose_dict.get("y", 0.0)
        yaw = pose_dict.get("yaw", 0.0)
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation = Quaternion(*q)
        return pose

    def _send_goal(self, pose: PoseStamped) -> bool:
        goal = MoveBaseGoal()
        pose.header.stamp = rospy.Time.now()
        goal.target_pose = pose
        self._goal_client.send_goal(goal)
        finished = self._goal_client.wait_for_result(rospy.Duration(self._goal_timeout))
        
        if not finished:
            self._goal_client.cancel_goal()
            return False
        return self._goal_client.get_state() == actionlib.GoalStatus.SUCCEEDED

    def _force_recovery(self):
        rospy.logwarn("kURTULACAĞIMMM....")
        twist = Twist()
        twist.linear.x = -0.2
        for _ in range(10): 
            self._cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        twist.linear.x = 0.0
        twist.angular.z = 0.8
        for _ in range(10): 
            self._cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        self._cmd_vel_pub.publish(Twist()) # Dur

    def _wiggle_robot(self):
        """
        QR Okunamadığında 'Wiggle' (Küçük titreme/açı değiştirme) hareketi yapar.
        Ödev Madde 5 gereksinimi: 'Küçük açı düzeltme'.
        """
        rospy.loginfo("O ZAMAAAAAN DANSS...")
        twist = Twist()

        twist.angular.z = 0.5
        for _ in range(5): 
            self._cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        

        self._cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)

        twist.angular.z = -0.5
        for _ in range(10):
            self._cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
            
        self._cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)

    def _try_goal_with_retry(self, pose: PoseStamped) -> bool:
        for attempt in range(self._max_retries + 1):
            if self._send_goal(pose): return True
            if attempt < self._max_retries:
                rospy.logwarn(f"Hedefe varılamadı. Kurtarma Denemesi {attempt+1}...")
                self._force_recovery()
        return False

    def _scan_qr(self) -> str:
        try:
            resp = self._qr_client()
            return resp.message if resp.success else ""
        except: return ""

    def process_room(self, room_name: str):
        waypoints = self._mission["waypoints"].get(room_name)
        if not waypoints:
            rospy.logwarn(f"HATA: {room_name} bilgisi mission.yaml içinde yok!")
            return

        rospy.loginfo(f"\n--- GÖREV BAŞLIYOR: {room_name} ---")
        
        rospy.loginfo(f" {room_name} girişine gidiliyor...")
        if not self._try_goal_with_retry(self._pose_from_dict(waypoints["entry_goal"])):
            rospy.logerr("Girişe varılamadı! FAIL.")
            self._log_report(room_name, "FAIL", "entry_failed", 0)
            return

        rospy.loginfo("QR Doğrulaması yapılıyor...")
        rospy.sleep(0.5)
        
        qr_expected = waypoints.get("qr_expected", "").strip()
        qr_read = ""
        verification_success = False
        
        for attempt in range(3):
            qr_read = self._scan_qr().strip()
            
            if qr_read == qr_expected:
                verification_success = True
                break
            
            if attempt < 2:
                rospy.logwarn(f"QR Okunamadı veya Eşleşmedi (Deneme {attempt+1}/")
                self._wiggle_robot()
            
        status, detail, duration = "SKIPPED", "qr_error", 0.0

        if verification_success:
            rospy.loginfo(f"QR'I GÖRDÜM: {qr_read}")
            
            rospy.loginfo("Temizlik Turu BAŞLASINNN...")
            start_t = time.time()
            success = True
            
            for i, pt in enumerate(waypoints.get("cleaning_goals", [])):
                if (time.time() - start_t) > self._room_timeout: 
                    rospy.logwarn("Oda süresi doldu!")
                    success = False; break
                
                rospy.loginfo(f"Temizlik Noktası {i+1}...")
                if not self._try_goal_with_retry(self._pose_from_dict(pt)): 
                    success = False; break
            
            duration = time.time() - start_t
            status = "SUCCESS" if success else "FAIL"
            detail = "done" if success else "incomplete"
            
            self._try_goal_with_retry(self._pose_from_dict(waypoints["entry_goal"]))
        else:
            if not qr_read:
                rospy.logerr("QR Hiç Bulunamadı.")
                detail = "qr_not_found"
            else:
                rospy.logerr(f"YANLIŞ ODA! Okunan: {qr_read}, Beklenen: {qr_expected}")
                detail = "qr_mismatch"

        self._log_report(room_name, status, detail, duration)

    def _log_report(self, room, status, detail, duration):
        line = f"{room},{status},{detail},{duration:.1f}"
        self._reports.append(line)
        self._report_pub.publish(String(data=line))
        rospy.loginfo(f"Rapor Eklendi: {line}")
        
        with open(self._report_path, "w", encoding="utf-8") as f:
            for r in self._reports: f.write(r + "\n")

    def run_menu(self):
        while not rospy.is_shutdown():
            print("\n" + "="*40)
            print("ROBOT SÜPÜRGE KONTROL PANELİ")
            print("1. OTONOM MOD")
            print("2. MANUEL MOD")
            print("3. ÇIKIŞ")
            print("="*40)
            
            choice = input("Seçiminiz (1/2/3): ").strip()
            
            if choice == '1':
                self.run_autonomous()
                break 
            elif choice == '2':
                self.run_manual_loop()
                break 
            elif choice == '3':
                print("Çıkış yapılıyor...")
                break
            else:
                print("Geçersiz seçim!")

    def run_autonomous(self):
        rospy.loginfo("OTONOM MOD BAŞLATILIYOR...")
        target_rooms = self._mission["rooms"]
        for room in target_rooms:
            if rospy.is_shutdown(): break
            self.process_room(room)
        rospy.loginfo(f"GÖREV BİTTİ. Rapor: {self._report_path}")

    def run_manual_loop(self):
        rospy.loginfo("MANUEL MOD BAŞLATILIYOR...")
        available_rooms = list(self._mission["waypoints"].keys())
        
        while not rospy.is_shutdown():
            print("\n--- MEVCUT ODALAR ---")
            for i, r in enumerate(available_rooms):
                print(f"[{i+1}] {r}")
            print("[0] Bitir")
            
            user_input = input("Gitmek istediğiniz odaların numaraları (örn: 2 1): ").strip()
            
            if user_input == '0':
                break
            
            indices = user_input.split()
            valid_rooms = []
            
            for idx_str in indices:
                if idx_str.isdigit():
                    idx = int(idx_str) - 1
                    if 0 <= idx < len(available_rooms):
                        valid_rooms.append(available_rooms[idx])
            
            if valid_rooms:
                rospy.loginfo(f"Seçilen Rota: {valid_rooms}")
                for room in valid_rooms:
                    self.process_room(room)
                print("\nSeçilen tur tamamlandı.")
            else:
                print("Geçersiz giriş!")

def main():
    rospy.init_node("task_manager_node")
    manager = TaskManager()
    time.sleep(1)
    
    try:
        manager.run_menu()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()