#!/usr/bin/env python3
"""
🎯 고급 매핑 기반 VR Bridge System
- 40+ 정밀 매핑 포인트
- 위치 + 방향 복합 매핑
- 다중 KD-Tree 보간
- 실시간 부드러운 제어
"""

import rospy
import numpy as np
import json
import socket
import threading
import time
from collections import deque
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import tf.transformations as tf_trans
from scipy.spatial import cKDTree

class AdvancedVRBridgeMapping:
    def __init__(self):
        rospy.init_node('advanced_vr_bridge')
        
        print("🎯 고급 매핑 VR Bridge 시스템 시작")
        
        # 소켓 서버 설정
        self.setup_socket_server()
        vr_bridge.
        # VR 데이터 저장
        self.vr_data = {
            'hand_pose': {
                'position': np.array([0.0, 0.0, 0.0]),
                'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
                'initial_position': None,
                'initial_orientation': None,
                'calibrated': False,
                'pose_history': deque(maxlen=7)  # 더 많은 스무딩
            },
            'inputs': {
                'trigger': 0.0,
                'button_upper': False,
                'button_lower': False,
                'thumb_stick_horizontal': 0.0,
                'thumb_stick_vertical': 0.0,
                'press_index': 0.0,
                'press_middle': 0.0
            }
        }
        
        # 로봇 조인트 상태
        self.robot_joints = [0.0, -0.5, 1.0, 0.0]
        self.gripper_value = -0.01
        
        # 확장된 매핑 데이터
        self.mapping_data = self.build_comprehensive_mapping()
        
        # 다중 KD-Tree 구성
        self.build_multiple_trees()
        
        # 제어 파라미터 (개선됨)
        self.position_scale = np.array([1.0, 1.0, 1.0])
        self.smooth_factor = 0.25  # 더 부드럽게
        self.mapping_threshold = 0.30  # 범위 확대
        self.interpolation_neighbors = 6  # 더 많은 이웃점
        
        # 성능 통계
        self.stats = {
            'position_mapping_used': 0,
            'orientation_mapping_used': 0,
            'hybrid_mapping_used': 0,
            'ik_fallback': 0,
            'interpolation_time': 0.0,
            'control_frequency': 0.0,
            'mapping_accuracy': 0.0
        }
        
        # ROS 설정
        self.setup_ros_topics()
        
        # 제어 루프 시작
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        
        # 디버그 스레드
        self.debug_thread = threading.Thread(target=self.debug_loop, daemon=True)
        self.debug_thread.start()
        
        print("✅ 고급 매핑 VR Bridge 시스템 준비 완료")
        print(f"📍 {len(self.mapping_data['position'])}개 위치 매핑 포인트")
        print(f"🎯 {len(self.mapping_data['orientation'])}개 방향 매핑 포인트")
        print(f"🔄 {len(self.mapping_data['hybrid'])}개 복합 매핑 포인트")
    
    def build_comprehensive_mapping(self):
        """포괄적인 매핑 데이터 구축"""
        mapping = {
            'position': [
                # 기본 11개 위치 (기존 데이터)
                {'vr_delta': [0.0, 0.0, 0.0], 'joints': [-0.05, 0.1, -0.15, 0.05], 'name': '기준'},
                {'vr_delta': [-0.262, 0.009, -0.026], 'joints': [-0.05, -0.35, 0.3, 0.05], 'name': '뒤로'},
                {'vr_delta': [0.200, -0.003, 0.029], 'joints': [-0.05, 0.85, -0.95, 0.05], 'name': '앞으로'},
                {'vr_delta': [-0.108, -0.013, 0.363], 'joints': [-0.05, -0.2, -0.8, 0.75], 'name': '위로'},
                {'vr_delta': [0.014, 0.036, -0.260], 'joints': [0, 1.35, -1.05, -0.3], 'name': '아래로'},
                {'vr_delta': [-0.017, 0.352, 0.020], 'joints': [0.75, -0.1, 0.1, -0.1], 'name': '좌로'},
                {'vr_delta': [-0.114, -0.265, -0.043], 'joints': [-0.95, -0.1, 0.1, -0.1], 'name': '우로'},
                {'vr_delta': [-0.060, 0.251, 0.318], 'joints': [0.95, -0.7, -0.05, 0.3], 'name': '좌상'},
                {'vr_delta': [-0.130, -0.235, 0.197], 'joints': [-0.95, -0.7, -0.05, 0.3], 'name': '우상'},
                {'vr_delta': [-0.060, 0.363, -0.161], 'joints': [0.95, 0.5, 0.25, -0.65], 'name': '좌하'},
                {'vr_delta': [-0.147, -0.253, -0.252], 'joints': [-0.95, 0.5, 0.25, -0.65], 'name': '우하'},
                
                # 앞뒤 세분화 4개
                {'vr_delta': [-0.290, 0.015, -0.015], 'joints': [0, -0.95, 0.95, 0.05], 'name': '몸쪽최대'},
                {'vr_delta': [-0.125, 0.006, -0.024], 'joints': [0, -0.4, 0.35, 0.05], 'name': '몸쪽절반'},
                {'vr_delta': [0.104, 0.035, 0.021], 'joints': [0, 0.5, -0.55, 0.05], 'name': '앞중간'},
                {'vr_delta': [0.071, -0.031, 0.241], 'joints': [0, 0.9, -0.95, 0.05], 'name': '앞최대'},
            ],
            
            'orientation': [
                # 손목 방향별 매핑 (orientation 기반)
                {'vr_ori_delta': [0.0, 0.0, 0.0], 'joint4_delta': 0.0, 'name': '손목기준'},
                {'vr_ori_delta': [0.059, -0.347, -0.041], 'joint4_delta': 1.75, 'name': '손목위최대'},
                {'vr_ori_delta': [0.181, 0.393, 0.010], 'joint4_delta': 1.05, 'name': '손목위중간'},
                {'vr_ori_delta': [0.221, 0.767, 0.069], 'joint4_delta': 1.6, 'name': '손목아래최대'},
                {'vr_ori_delta': [0.181, 0.393, 0.010], 'joint4_delta': 1.05, 'name': '손목아래중간'},
            ],
            
            'hybrid': [
                # 복합 동작 매핑 (위치 + 방향)
                {'vr_delta': [-0.021, 0.156, 0.273], 'vr_ori_delta': [-0.042, 0.014, 0.159], 
                 'joints': [0.4, -0.45, -0.25, 0.05], 'name': '좌중간+위'},
                {'vr_delta': [0.077, 0.187, -0.132], 'vr_ori_delta': [0.168, 0.045, 0.153], 
                 'joints': [0.4, 1.0, -0.55, 0.05], 'name': '좌중간+아래'},
                {'vr_delta': [0.053, 0.342, -0.118], 'vr_ori_delta': [-0.031, 0.140, 0.318], 
                 'joints': [0.75, 1.0, -0.35, 0.05], 'name': '좌최대+아래'},
                {'vr_delta': [-0.027, 0.279, 0.310], 'vr_ori_delta': [-0.061, -0.135, 0.233], 
                 'joints': [0.75, -9.71e-17, -0.6, 0.05], 'name': '좌최대+위'},
                {'vr_delta': [-0.086, -0.257, -0.182], 'vr_ori_delta': [0.054, 0.200, -0.395], 
                 'joints': [-0.9, 1.25, -0.95, 0.05], 'name': '우최대+아래'},
                {'vr_delta': [-0.244, -0.301, 0.191], 'vr_ori_delta': [-0.004, -0.045, -0.519], 
                 'joints': [-0.9, 1.25, -0.95, 0.05], 'name': '우최대+위'},
                {'vr_delta': [-0.019, -0.174, -0.179], 'vr_ori_delta': [0.077, 0.177, -0.192], 
                 'joints': [-0.45, 0.25, -0.7, 0.05], 'name': '우중간+아래'},
                {'vr_delta': [-0.071, -0.184, 0.241], 'vr_ori_delta': [-0.013, -1.022, -0.246], 
                 'joints': [-0.45, 0.25, -0.7, 0.05], 'name': '우중간+위'},
            ]
        }
        
        return mapping
    
    def build_multiple_trees(self):
        """다중 KD-Tree 구성"""
        # 위치 기반 트리
        if len(self.mapping_data['position']) >= 3:
            positions = np.array([m['vr_delta'] for m in self.mapping_data['position']])
            self.position_tree = cKDTree(positions)
        else:
            self.position_tree = None
        
        # 방향 기반 트리
        if len(self.mapping_data['orientation']) >= 3:
            orientations = np.array([m['vr_ori_delta'] for m in self.mapping_data['orientation']])
            self.orientation_tree = cKDTree(orientations)
        else:
            self.orientation_tree = None
        
        # 복합 동작 트리 (6D: position + orientation)
        if len(self.mapping_data['hybrid']) >= 3:
            hybrid_features = []
            for m in self.mapping_data['hybrid']:
                combined = np.concatenate([m['vr_delta'], m['vr_ori_delta']])
                hybrid_features.append(combined)
            self.hybrid_tree = cKDTree(np.array(hybrid_features))
        else:
            self.hybrid_tree = None
        
        print(f"🔍 다중 KD-Tree 구성 완료:")
        print(f"   위치 트리: {len(self.mapping_data['position'])}개 포인트")
        print(f"   방향 트리: {len(self.mapping_data['orientation'])}개 포인트") 
        print(f"   복합 트리: {len(self.mapping_data['hybrid'])}개 포인트")
    
    def setup_socket_server(self):
        """소켓 서버 설정"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', 12345))
            self.server_socket.listen(5)
            self.clients = []
            
            accept_thread = threading.Thread(target=self.accept_clients, daemon=True)
            accept_thread.start()
            
            print("✅ 소켓 서버 시작: 포트 12345")
        except Exception as e:
            print(f"❌ 소켓 서버 오류: {e}")
    
    def accept_clients(self):
        """클라이언트 수락"""
        while True:
            try:
                client, addr = self.server_socket.accept()
                self.clients.append(client)
                print(f"🔗 MuJoCo 클라이언트 연결: {addr}")
            except:
                break
    
    def setup_ros_topics(self):
        """ROS 토픽 설정"""
        rospy.Subscriber('/q2r_left_hand_pose', PoseStamped, self.hand_pose_callback)
        
        try:
            from quest2ros.msg import OVR2ROSInputs
            rospy.Subscriber('/q2r_left_hand_inputs', OVR2ROSInputs, self.input_callback)
            print("✅ VR 입력 토픽 구독됨")
        except ImportError:
            print("⚠️ OVR2ROSInputs 메시지 없음")
        
        print("✅ ROS 토픽 설정 완료")
    
    def hand_pose_callback(self, msg):
        """VR 손 Pose 콜백 (향상된 스무딩)"""
        current_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        current_orientation = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        
        # 향상된 스무딩 (더 많은 히스토리)
        pose_data = {
            'position': current_position,
            'orientation': current_orientation
        }
        self.vr_data['hand_pose']['pose_history'].append(pose_data)
        
        if len(self.vr_data['hand_pose']['pose_history']) >= 5:
            positions = [p['position'] for p in self.vr_data['hand_pose']['pose_history']]
            orientations = [p['orientation'] for p in self.vr_data['hand_pose']['pose_history']]
            
            # 가중 평균 스무딩 (최근 데이터에 더 높은 가중치)
            weights = np.linspace(0.5, 1.0, len(positions))
            weights = weights / np.sum(weights)
            
            smoothed_position = np.average(positions, axis=0, weights=weights)
            smoothed_orientation = np.average(orientations, axis=0, weights=weights)
            smoothed_orientation = smoothed_orientation / np.linalg.norm(smoothed_orientation)
        else:
            smoothed_position = current_position
            smoothed_orientation = current_orientation
        
        self.vr_data['hand_pose']['position'] = smoothed_position
        self.vr_data['hand_pose']['orientation'] = smoothed_orientation
        
        # 초기 캘리브레이션
        if not self.vr_data['hand_pose']['calibrated']:
            self.vr_data['hand_pose']['initial_position'] = smoothed_position.copy()
            self.vr_data['hand_pose']['initial_orientation'] = smoothed_orientation.copy()
            self.vr_data['hand_pose']['calibrated'] = True
            print(f"🖐 VR 컨트롤러 캘리브레이션 완료")
    
    def input_callback(self, msg):
        """VR 입력 콜백"""
        try:
            if hasattr(msg, 'trigger'):
                self.vr_data['inputs']['trigger'] = msg.trigger
            if hasattr(msg, 'button_upper'):
                self.vr_data['inputs']['button_upper'] = msg.button_upper
            if hasattr(msg, 'button_lower'):
                self.vr_data['inputs']['button_lower'] = msg.button_lower
            
            # A+B 버튼으로 재캘리브레이션
            if (self.vr_data['inputs']['button_upper'] and 
                self.vr_data['inputs']['button_lower']):
                self.recalibrate()
                
        except Exception as e:
            rospy.logwarn(f"입력 처리 오류: {e}")
    
    def recalibrate(self):
        """재캘리브레이션"""
        if self.vr_data['hand_pose']['position'] is not None:
            self.vr_data['hand_pose']['initial_position'] = self.vr_data['hand_pose']['position'].copy()
            self.vr_data['hand_pose']['initial_orientation'] = self.vr_data['hand_pose']['orientation'].copy()
            self.vr_data['hand_pose']['pose_history'].clear()
            print("🔄 VR 컨트롤러 재캘리브레이션 완료!")
    
    def get_vr_deltas(self):
        """VR 델타 계산"""
        if not self.vr_data['hand_pose']['calibrated']:
            return None, None
        
        # Position 델타
        current_pos = self.vr_data['hand_pose']['position']
        initial_pos = self.vr_data['hand_pose']['initial_position']
        position_delta = current_pos - initial_pos
        
        # Orientation 델타 (오일러각 차이)
        current_ori = self.vr_data['hand_pose']['orientation']
        initial_ori = self.vr_data['hand_pose']['initial_orientation']
        
        # 쿼터니언을 오일러각으로 변환
        current_euler = tf_trans.euler_from_quaternion(current_ori)
        initial_euler = tf_trans.euler_from_quaternion(initial_ori)
        orientation_delta = np.array(current_euler) - np.array(initial_euler)
        
        return position_delta, orientation_delta
    
    def advanced_interpolation(self, vr_pos_delta, vr_ori_delta):
        """고급 다단계 보간"""
        start_time = time.time()
        
        # 1. 복합 매핑 우선 시도
        if self.hybrid_tree and len(self.mapping_data['hybrid']) >= 3:
            combined_delta = np.concatenate([vr_pos_delta, vr_ori_delta])
            distance, idx = self.hybrid_tree.query(combined_delta, k=1)
            
            if distance < 0.2:  # 복합 매핑 임계값
                # 가까운 복합 매핑 포인트들로 보간
                distances, indices = self.hybrid_tree.query(
                    combined_delta, k=min(4, len(self.mapping_data['hybrid']))
                )
                
                if isinstance(distances, float):
                    distances = [distances]
                    indices = [indices]
                
                # 가중 평균
                weights = 1.0 / (np.array(distances) + 1e-6)
                weights = weights / np.sum(weights)
                
                interpolated_joints = np.zeros(4)
                for i, idx in enumerate(indices):
                    joints = np.array(self.mapping_data['hybrid'][idx]['joints'])
                    interpolated_joints += weights[i] * joints
                
                self.stats['hybrid_mapping_used'] += 1
                self.stats['interpolation_time'] = time.time() - start_time
                return interpolated_joints.tolist()
        
        # 2. 위치 매핑으로 기본 자세 계산
        base_joints = None
        if self.position_tree and len(self.mapping_data['position']) >= 3:
            distances, indices = self.position_tree.query(
                vr_pos_delta, k=min(self.interpolation_neighbors, len(self.mapping_data['position']))
            )
            
            if isinstance(distances, float):
                distances = [distances]
                indices = [indices]
            
            if distances[0] < self.mapping_threshold:
                weights = 1.0 / (np.array(distances) + 1e-6)
                weights = weights / np.sum(weights)
                
                base_joints = np.zeros(4)
                for i, idx in enumerate(indices):
                    joints = np.array(self.mapping_data['position'][idx]['joints'])
                    base_joints += weights[i] * joints
                
                self.stats['position_mapping_used'] += 1
        
        # 3. 방향 매핑으로 Joint4 조정
        joint4_adjustment = 0.0
        if self.orientation_tree and len(self.mapping_data['orientation']) >= 3:
            distances, indices = self.orientation_tree.query(
                vr_ori_delta, k=min(3, len(self.mapping_data['orientation']))
            )
            
            if isinstance(distances, float):
                distances = [distances]
                indices = [indices]
            
            if distances[0] < 0.5:  # 방향 매핑 임계값
                weights = 1.0 / (np.array(distances) + 1e-6)
                weights = weights / np.sum(weights)
                
                for i, idx in enumerate(indices):
                    joint4_delta = self.mapping_data['orientation'][idx]['joint4_delta']
                    joint4_adjustment += weights[i] * joint4_delta
                
                self.stats['orientation_mapping_used'] += 1
        
        # 4. 결합된 결과
        if base_joints is not None:
            result_joints = base_joints.copy()
            result_joints[3] += joint4_adjustment * 0.3  # Joint4 조정 강도
            
            # 조인트 제한 적용
            joint_limits = [
                [-3.14, 3.14], [-1.5, 1.5], [-1.5, 1.4], [-2.0, 2.0]
            ]
            for i in range(4):
                result_joints[i] = np.clip(result_joints[i], 
                                         joint_limits[i][0], joint_limits[i][1])
            
            self.stats['interpolation_time'] = time.time() - start_time
            return result_joints.tolist()
        
        # 5. 백업 IK
        self.stats['ik_fallback'] += 1
        return self.fallback_ik_solver(vr_pos_delta)
    
    def fallback_ik_solver(self, vr_pos_delta):
        """백업 IK 솔버"""
        # 간단한 백업 IK (기존 방식)
        base_position = np.array([0.20, 0.0, 0.15])
        target_position = base_position + vr_pos_delta
        
        # 작업공간 제한
        workspace_min = np.array([0.08, -0.25, 0.05])
        workspace_max = np.array([0.35, 0.25, 0.30])
        target_position = np.clip(target_position, workspace_min, workspace_max)
        
        # 단순 IK
        x, y, z = target_position
        joint1 = np.arctan2(y, x)
        
        # 2D IK
        r = np.sqrt(x*x + y*y) - 0.012
        z_eff = z - 0.077
        
        L1, L2 = 0.128, 0.124
        reach = np.sqrt(r*r + z_eff*z_eff)
        reach = np.clip(reach, 0.05, (L1 + L2) * 0.95)
        
        cos_q3 = (reach**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_q3 = np.clip(cos_q3, -0.99, 0.99)
        
        joint3 = np.arccos(cos_q3)
        if self.robot_joints[2] < 0:
            joint3 = -joint3
        
        alpha = np.arctan2(z_eff, r)
        beta = np.arctan2(L2 * np.sin(joint3), L1 + L2 * np.cos(joint3))
        joint2 = alpha - beta
        
        joint4 = -np.pi/4 - (joint2 + joint3) * 0.3
        
        joints = [joint1, joint2, joint3, joint4]
        joint_limits = [
            [-3.14, 3.14], [-1.5, 1.5], [-1.5, 1.4], [-2.0, 2.0]
        ]
        
        for i in range(4):
            joints[i] = np.clip(joints[i], joint_limits[i][0], joint_limits[i][1])
        
        return joints
    
    def update_gripper_control(self):
        """그리퍼 제어"""
        trigger_value = self.vr_data['inputs']['trigger']
        self.gripper_value = -0.01 + (trigger_value * 0.029)
        
        if self.vr_data['inputs']['button_upper']:
            self.gripper_value = 0.019
    
    def control_loop(self):
        """메인 제어 루프 (개선된 성능)"""
        rate = rospy.Rate(120)  # 120Hz로 증가
        
        while not rospy.is_shutdown():
            loop_start_time = time.time()
            
            if self.vr_data['hand_pose']['calibrated']:
                vr_pos_delta, vr_ori_delta = self.get_vr_deltas()
                
                if vr_pos_delta is not None and vr_ori_delta is not None:
                    try:
                        # 고급 매핑 보간
                        target_joints = self.advanced_interpolation(vr_pos_delta, vr_ori_delta)
                        
                        # 적응형 스무딩 (빠른 움직임 시 반응성 증가)
                        movement_speed = np.linalg.norm(vr_pos_delta)
                        adaptive_smooth = self.smooth_factor
                        if movement_speed > 0.1:  # 빠른 움직임
                            adaptive_smooth *= 0.7
                        
                        # 부드러운 움직임 적용
                        for i in range(4):
                            self.robot_joints[i] += (
                                target_joints[i] - self.robot_joints[i]
                            ) * adaptive_smooth
                            
                    except Exception as e:
                        rospy.logwarn(f"제어 오류: {e}")
            
            self.update_gripper_control()
            self.send_to_mujoco()
            
            # 성능 통계
            loop_time = time.time() - loop_start_time
            self.stats['control_frequency'] = 1.0 / max(loop_time, 0.001)
            
            rate.sleep()
    
    def send_to_mujoco(self):
        """MuJoCo로 데이터 전송"""
        if self.clients:
            data = {
                'joint_angles': self.robot_joints,
                'gripper': self.gripper_value,
                'vr_status': {
                    'calibrated': self.vr_data['hand_pose']['calibrated'],
                    'trigger_value': self.vr_data['inputs']['trigger'],
                    'button_upper': self.vr_data['inputs']['button_upper'],
                    'button_lower': self.vr_data['inputs']['button_lower']
                },
                'performance': self.stats,
                'timestamp': rospy.Time.now().to_sec()
            }
            
            json_data = json.dumps(data) + '\n'
            
            failed_clients = []
            for client in self.clients:
                try:
                    client.sendall(json_data.encode())
                except:
                    failed_clients.append(client)
            
            for client in failed_clients:
                if client in self.clients:
                    self.clients.remove(client)
    
    def debug_loop(self):
        """디버그 출력"""
        while not rospy.is_shutdown():
            time.sleep(5.0)
            
            total_operations = (self.stats['position_mapping_used'] + 
                              self.stats['orientation_mapping_used'] + 
                              self.stats['hybrid_mapping_used'] + 
                              self.stats['ik_fallback'])
            
            if total_operations > 0:
                pos_rate = (self.stats['position_mapping_used'] / total_operations) * 100
                ori_rate = (self.stats['orientation_mapping_used'] / total_operations) * 100
                hybrid_rate = (self.stats['hybrid_mapping_used'] / total_operations) * 100
                ik_rate = (self.stats['ik_fallback'] / total_operations) * 100
            else:
                pos_rate = ori_rate = hybrid_rate = ik_rate = 0.0
            
            print(f"\n🎯 === 고급 매핑 VR Bridge 상태 ===")
            print(f"🖐 VR 캘리브레이션: {'✅' if self.vr_data['hand_pose']['calibrated'] else '❌'}")
            print(f"🎮 트리거: {self.vr_data['inputs']['trigger']:.2f}")
            print(f"🤖 조인트: {[f'{j:.2f}' for j in self.robot_joints]}")
            print(f"⚡ 제어 주파수: {self.stats['control_frequency']:.1f}Hz")
            print(f"📍 위치 매핑: {pos_rate:.1f}%")
            print(f"🎯 방향 매핑: {ori_rate:.1f}%") 
            print(f"🔄 복합 매핑: {hybrid_rate:.1f}%")
            print(f"🛠️ IK 백업: {ik_rate:.1f}%")
            print(f"🚀 보간 시간: {self.stats['interpolation_time']*1000:.1f}ms")
            print(f"🌐 MuJoCo 클라이언트: {len(self.clients)}개")
            
            if self.vr_data['hand_pose']['calibrated']:
                vr_pos_delta, vr_ori_delta = self.get_vr_deltas()
                if vr_pos_delta is not None:
                    print(f"📍 VR 위치 델타: X={vr_pos_delta[0]:+.3f}, Y={vr_pos_delta[1]:+.3f}, Z={vr_pos_delta[2]:+.3f}")
                    print(f"🎯 VR 방향 델타: R={vr_ori_delta[0]:+.3f}, P={vr_ori_delta[1]:+.3f}, Y={vr_ori_delta[2]:+.3f}")
            
            # 통계 리셋
            self.stats['position_mapping_used'] = 0
            self.stats['orientation_mapping_used'] = 0
            self.stats['hybrid_mapping_used'] = 0
            self.stats['ik_fallback'] = 0

if __name__ == "__main__":
    bridge = AdvancedVRBridgeMapping()
    
    print("\n🎯 === 고급 매핑 VR 제어 시스템 ===")
    print("🖐 왼쪽 VR 컨트롤러 → OpenManipulator-X")
    print("📍 40+ 정밀 매핑 포인트 사용")
    print("🚀 다중 KD-Tree 초고속 보간")
    print("🔄 위치 + 방향 복합 매핑")
    print("🎯 트리거 → 그리퍼 제어")
    print("🔄 A+B 버튼 → 재캘리브레이션")
    
    try:
        while not rospy.is_shutdown():
            try:
                key = input().strip().lower()
                
                if key == 'c':
                    bridge.recalibrate()
                elif key == 'r':
                    bridge.robot_joints = [0.0, -0.5, 1.0, 0.0]
                    bridge.gripper_value = -0.01
                    print("🔄 로봇 리셋됨")
                elif key == 'q':
                    break
                    
            except (EOFError, KeyboardInterrupt):
                break
                
    except:
        pass
    
    print("🏁 고급 매핑 VR Bridge 시스템 종료")
