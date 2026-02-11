#!/usr/bin/env python3
import rospy
import actionlib
import numpy as np
import torch
import trimesh
import trimesh.transformations as tra
import ros_numpy
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf import TransformListener
from tf.transformations import quaternion_about_axis, quaternion_multiply, unit_vector, quaternion_from_matrix

from grasp_gen.grasp_server import GraspGenSampler, load_grasp_cfg
from grasp_gen.utils.point_cloud_utils import depth_and_segmentation_to_point_clouds, point_cloud_outlier_removal, filter_colliding_grasps
from grasp_gen.utils.meshcat_utils import create_visualizer, visualize_pointcloud, visualize_grasp, get_color_from_score
from grasp_gen.robot import get_gripper_info

from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorResult

class GraspGenWrapper():

    def __init__(self):
        rospy.init_node("graspgen_wrapper")
        self.server = actionlib.SimpleActionServer(
            "/pose_estimator/find_grasppose_graspgen",
            GenericImgProcAnnotatorAction,
            self.execute_cb,
            auto_start=False
        )
        rospy.loginfo("GraspGenWrapper initializing...")
        # Load GraspGen model config
        gripper_cfg_path = rospy.get_param("~gripper_config", "/GraspGen2004/code/GraspGenModels/checkpoints/graspgen_robotiq_2f_140.yml")
        self.grasp_cfg = load_grasp_cfg(gripper_cfg_path)
        self.grasp_sampler = GraspGenSampler(self.grasp_cfg)
        self.gripper_info = get_gripper_info(self.grasp_cfg.data.gripper_name)
        self.gripper_collision_mesh = self.gripper_info.collision_mesh
        self.visualizer = create_visualizer()

        # Depth scale
        self.depth_scale = 1000.0 # rospy.get_param("~depth_scale", 1000.0)
        self.cam_info = None

        self.server.start()
        rospy.loginfo("GraspGenWrapper server started")


    @staticmethod
    def process_point_cloud(pc, grasps, grasp_conf, pc_colors=None):
        """Process point cloud and grasps by centering them."""
        scores = get_color_from_score(grasp_conf, use_255_scale=True)
        print(f"Scores with min {grasp_conf.min():.3f} and max {grasp_conf.max():.3f}")

        # Ensure grasps have correct homogeneous coordinate
        grasps[:, 3, 3] = 1

        # Center point cloud and grasps
        T_subtract_pc_mean = tra.translation_matrix(-pc.mean(axis=0))
        pc_centered = tra.transform_points(pc, T_subtract_pc_mean)
        grasps_centered = np.array(
            [T_subtract_pc_mean @ np.array(g) for g in grasps.tolist()]
        )

        # Add red tint to colors if RGB data is available
        pc_colors_centered = pc_colors
        if pc_colors is not None:
            pc_colors_centered = pc_colors.copy().astype(np.float32)
            # Add red tint: increase red channel by 40% while keeping it in valid range
            pc_colors_centered[:, 0] = np.clip(pc_colors_centered[:, 0] * 1.4, 0, 255)
            pc_colors_centered = pc_colors_centered.astype(np.uint8)

        return pc_centered, grasps_centered, scores, T_subtract_pc_mean, pc_colors_centered
    
    @staticmethod
    def fit_plane_ransac(points_scene, confidence=0.95, num_iters=500, dist_thresh=0.01, points_object=None, object_dist=0.05):
        """ Find dominant plane in pointcloud with sample consensus.

        Fit plane ax + by + cz + d = 0 via simple RANSAC.

        :param points_scene: point array of Nx3 of the scene (without object)
        :type points_scene: np.ndarray

        :param confidence: Solution Confidence (in percent): Likelihood of all sampled points being inliers.
        :type confidence: float

        :param inlier_threshold: Max. distance of a point from the plane to be considered an inlier (in meters)
        :type dist_thresh: float

        :param points_object: point array of Nx3 of the object
        :type points_object: np.ndarray

        :param object_dist: fMax distance an object may have to the plane in meters
        :type object_dist: float

        :return: (normal, d, inlier_mask)
        :rtype: tuple (np.ndarray, float, np.ndarray)
        """
        best_plane_vector = None
        best_plane_d = None
        best_msk = None

        N = points_scene.shape[0]
        m = 3  # sample size
        eps = m / N  # initial assumption of inlier propability

        it_count = 0
        while (1 - (1 - eps ** m) ** num_iters < confidence) and it_count<num_iters*1.2: # allow maximum 20% more iterations
            it_count +=1
            ids = np.random.choice(N, 3, replace=False)
            p1, p2, p3 = points_scene[ids]

            # plane normal
            n = np.cross(p2 - p1, p3 - p1)
            norm = np.linalg.norm(n)
            if norm < 1e-6:
                continue
            n = n / norm
            d = -np.dot(n, p1)

            # distance of all points to plane
            dist = np.abs(points_scene @ n + d)
            inliers_msk = dist < dist_thresh
            inliers = np.sum(inliers_msk)
            
            percentage_inliers = inliers / N

            # sanity check if available, objects cannot fly, one needs to be on the plane
            if points_object is not None:
                dists_to_object = np.abs(points_object @ n + d)
                sanity_flag = np.any(dists_to_object < object_dist)
            else:
                sanity_flag = True

            if percentage_inliers > eps and sanity_flag:
                eps = percentage_inliers
                best_plane_vector = n
                best_plane_d = d
                best_msk = inliers_msk

        return best_plane_vector, best_plane_d, best_msk

    def get_cam_info(self):
        cam_info_topic = rospy.get_param("~camera_info_topic", '/hsrb/head_rgbd_sensor/depth_registered/camera_info')
        self.cam_info = rospy.wait_for_message(cam_info_topic, CameraInfo, 10.0)
        print(self.cam_info)


    def execute_cb(self, goal):
        result = GenericImgProcAnnotatorResult()
        try:
            # get cameera intrinsics
            self.get_cam_info()
            intrinsics = np.array(self.cam_info.K).reshape(3, 3)
            fx = intrinsics[0, 0]
            fy = intrinsics[1, 1]
            cx = intrinsics[0, 2]
            cy = intrinsics[1, 2]
            print(f"fx: {fx}")
            print(f"fy: {fy}")
            print(f"cx: {cx}")
            print(f"cy: {cy}")

            # Convert depth and label to numpy
            depth_image = ros_numpy.numpify(goal.depth)
            depth = depth_image.astype(np.float32) / self.depth_scale
            label = ros_numpy.numpify(goal.mask_detections[0])
            rgb = ros_numpy.numpify(goal.rgb)

            # Create point clouds from depth and segmentation
            # GraspGen wants only one object to be in the segmentation mask
            #target_id = 1
            #label[label != target_id] = 0
            # !!! within gp the given mask will be binary!! So Object will have ID = 1 and background is = 0

            # Create point clouds
            pc_scene, pc_object, pc_colors_scene, pc_colors_object = depth_and_segmentation_to_point_clouds(
                depth_image=depth,
                segmentation_mask=label,
                fx=fx, fy=fy, cx=cx, cy=cy,
                rgb_image=rgb,
                target_object_id=1,
                remove_object_from_scene=True
            )

            # Filter outliers
            pc_object_torch = torch.from_numpy(pc_object)
            pc_filtered, pc_removed = point_cloud_outlier_removal(pc_object_torch)
            pc_filtered = pc_filtered.numpy()

            visualize_pointcloud(self.visualizer, "obj-pc-cam-frame", pc_object)

            # Grasp inference
            grasps_inferred, grasp_conf_inferred = GraspGenSampler.run_inference(
                pc_filtered,
                self.grasp_sampler,
                grasp_threshold=0.8,
                num_grasps=8000,
                topk_num_grasps=4000
            )

            # Convert to numpy
            grasp_conf_inferred = grasp_conf_inferred.cpu().numpy()
            grasps_inferred = grasps_inferred.cpu().numpy()
            grasps_inferred[:, 3, 3] = 1


            # Process point clouds and grasps for consistent coordinate frame
            pc_centered, grasps_centered, scores, T_center, object_colors_centered = (
                self.process_point_cloud(
                    pc_filtered, grasps_inferred, grasp_conf_inferred, pc_colors_object
                )
            )
            scene_pc_centered = tra.transform_points(pc_scene, T_center)

            # Add red tint to scene colors if RGB data is available
            scene_colors_centered = pc_colors_scene
            if pc_colors_scene is not None:
                scene_colors_centered = pc_colors_scene.copy().astype(np.float32)
                # Add red tint: increase red channel by 40% while keeping it in valid range
                scene_colors_centered[:, 0] = np.clip(scene_colors_centered[:, 0] * 1.4, 0, 255)
                scene_colors_centered = scene_colors_centered.astype(np.uint8)

            # Downsample scene point cloud for faster collision checking (keep full resolution for visualization)
            if len(scene_pc_centered) > 8192:
                indices = np.random.choice(
                    len(scene_pc_centered), 8192, replace=False
                )
                scene_pc_downsampled = scene_pc_centered[indices]
                print(
                    f"Downsampled scene point cloud from {len(scene_pc_centered)} to {len(scene_pc_downsampled)} points for collision checking"
                )
            else:
                scene_pc_downsampled = scene_pc_centered
                print(
                    f"Scene point cloud has {len(scene_pc_centered)} points (no downsampling needed)"
                )

            # Filter collision grasps using downsampled scene
            collision_free_mask = filter_colliding_grasps(
                scene_pc=scene_pc_downsampled,
                grasp_poses=grasps_centered,
                gripper_collision_mesh=self.gripper_collision_mesh,
                collision_threshold=0.01,
            )

            # Filter grasps to only collision-free ones
            collision_free_grasps = grasps_centered[collision_free_mask]
            collision_free_scores = grasp_conf_inferred[collision_free_mask]
            collision_free_colors = scores[collision_free_mask]


            ################## grasp filtering ###############################################################################################################################
            # trafo back to camera frame
            T_center_inv = np.linalg.inv(T_center)

            # extract table normal
            plane_normal, plane_d, plane_points_msk = self.fit_plane_ransac(points_scene=pc_scene, points_object=pc_object)
            if plane_normal is None:
                raise ValueError("Could not find the plane")
            # print(f"plane model is:\nn:{plane_normal}\nd:{plane_d}")

            # ensure normal points toward camera
            # plane point closest to origin
            p0 = -plane_d * plane_normal # p0 points from origin toward plane along normal vector
            if np.dot(plane_normal, -p0) < 0: # -p0 points from plane to origin (camera), so if plane_nomal . -p0 negative -> plane_normal points NOT to camera
                plane_normal = -plane_normal
                plane_d = -plane_d

            # masks for each filter aspect
            N = collision_free_grasps.shape[0]
            dist_mask = np.ones(N, dtype=bool)
            plane_mask = np.ones(N, dtype=bool)
            cam_mask = np.ones(N, dtype=bool)

            # filtering for grasps geometrically
            grasps_above_and_toward_table = []
            for idx, g_c in enumerate(collision_free_grasps):
                g = T_center_inv @ g_c # g_c: centered grasp, g is in camera frame

                grasp_pos = g[:3, 3]

                dist_to_plane = np.dot(plane_normal, grasp_pos) + plane_d
                # print(dist_to_plane)

                approach_dir = g[:3, 2]   # Z axis of grasp

                # cam_to_grasp_dir = grasp_pos / np.linalg.norm(grasp_pos) # direction vector towards grasp position
                # cam_to_grasp_dir = np.array([0, 0, 1]) # use just z axis, simpler
                cam_to_grasp_dir = - np.cross(np.array([1, 0, 0]), plane_normal) # -> gives (always) vector pointing toward the table roughly horizontally
                # sanity check:
                if cam_to_grasp_dir[2] < 0: # pointing from table toward robot
                    cam_to_grasp_dir = -cam_to_grasp_dir # reverse to point from robot toward table
                cam_to_grasp_dir = cam_to_grasp_dir/ np.linalg.norm(cam_to_grasp_dir)

                # distance must be sufficiently positive so above the table
                if dist_to_plane < 0.1:
                    dist_mask[idx] = False
                    continue
                # plane_normal points towards camera, approach should come from the side or from above -> dot-product should be negative!
                elif np.dot(approach_dir, plane_normal) > -0.1:
                    plane_mask[idx] = False
                    continue
                # we want it grasping the object with an approach forward, not against camera direction to avoid hollow grasps
                elif np.dot(approach_dir, cam_to_grasp_dir) < 0:
                    cam_mask[idx] = False
                    continue
                else:
                    grasps_above_and_toward_table.append(g_c)

            print(f"Dist filtering removed: {np.sum(~dist_mask)} grasps from {N}")
            print(f"Plane filtering removed: {np.sum(~plane_mask)} grasps from {N}")
            print(f"Cam filtering removed: {np.sum(~cam_mask)} grasps from {N}")
            
            full_mask = dist_mask&plane_mask&cam_mask
            grasp_filtered_score = collision_free_colors[full_mask]

            ################## grasp filtering end ###########################################################################################################################
            # apply gripper transformations due to gripper mismatch GraspGen vs Sasha
            sasha_grasps = []
            flag = True
            for grasp in grasps_above_and_toward_table: # these are centered grasps!
                T_center_inv = np.linalg.inv(T_center)
                if flag:
                    print("grasp in centered frame is:")
                    print(grasp)
                grasp_cf = T_center_inv @ grasp # this is now in camera frame
                if flag:
                    print("grasp in camera frame is:")
                    print(grasp_cf)


                # make sure grasps are well defined with y axis pointing upward:
                # camera up (negative Y in your setup)
                camera_up = np.array([0, -1, 0])

                y_axis = grasp_cf[:3, 1]   # gripper Y axis

                # if fingers point upward → flip 180° around approach axis
                if np.dot(y_axis, camera_up) > 0:
                    
                    grasp_cf = grasp_cf @ tra.rotation_matrix(np.pi, [0, 0, 1])
                    if flag:
                        print("Sent Grasp will be fliped around the z-axis")
                        print("grasp after flipping is:")
                        print(grasp_cf)

                # need to apply gripper offset along grippers z axis and turn it by 90° around its z axis
                # apply matrix from right to be in grasps coordinate system
                trafo_t = tra.translation_matrix([0, 0, 0.135])
                if flag:
                    print("trafo_t is")
                    print(trafo_t)
                    print("grasp_cf was:")
                    print(grasp_cf)
                grasp_cf = grasp_cf @ trafo_t
                if flag:
                    print("grasp_cf is after trafo_t:")
                    print(grasp_cf)

                trafo_R = tra.rotation_matrix(-np.pi/2, [0, 0, 1])
                if flag:
                    print("trafo_R is")
                    print(trafo_R)
                    print("grasp_cf was:")
                    print(grasp_cf)
                grasp_cf = grasp_cf @ trafo_R
                if flag:
                    print("grasp_cf is after trafo_R:")
                    print(grasp_cf)
                sasha_grasp = grasp_cf # verified in rviz against Graspgen documentation
                
                flag = False
                sasha_grasps.append(sasha_grasp)

            print("T_center_inv is:")
            print(T_center_inv)    
            
            # Convert to ROS poses
            grasp_poses = []
            for g in sasha_grasps:
                pose = Pose()
                pose.position.x = g[0][3]
                pose.position.y = g[1][3]
                pose.position.z = g[2][3]
                quat = tra.quaternion_from_matrix(g) # returns w, x, y, z
                pose.orientation.x = quat[1]
                pose.orientation.y = quat[2]
                pose.orientation.z = quat[3]
                pose.orientation.w = quat[0]
                grasp_poses.append(pose)

            print("Goal pose is:")
            print(grasp_poses[0])


            # Visualize scene point cloud - use RGB colors if available, otherwise use gray
            # if scene_colors_centered is not None:
            #     visualize_pointcloud(
            #         self.visualizer, "scene_pc", scene_pc_centered[~plane_points_msk], scene_colors_centered[~plane_points_msk], size=0.002
            #     )
            #     visualize_pointcloud(
            #         self.visualizer, "plane_pc", scene_pc_centered[plane_points_msk], scene_colors_centered[plane_points_msk], size=0.002
            #     )
            # else:
                # visualize_pointcloud(
                #     self.visualizer, "scene_pc", scene_pc_centered, [128, 128, 128], size=0.002
                # )
            visualize_pointcloud(
                self.visualizer, "plane_pc", scene_pc_centered[plane_points_msk], [230,230,230], size=0.002
            )

            # Visualize object point cloud - use RGB colors if available, otherwise use green
            if object_colors_centered is not None:
                visualize_pointcloud(
                    self.visualizer, "object_pc", pc_centered, object_colors_centered, size=0.0025
                )
            else:
                visualize_pointcloud(
                    self.visualizer, "object_pc", pc_centered, [0, 255, 0], size=0.0025
                )


            # Visualize collision-free grasps
            for i, (grasp, score) in enumerate(
                zip(grasps_above_and_toward_table[1:10], grasp_filtered_score[1:10])
            ):  
                visualize_grasp(
                    self.visualizer,
                    f"collision_free_grasps/{i:03d}/grasp",
                    grasp,
                    color=score,
                    gripper_name=self.grasp_cfg.data.gripper_name,
                    linewidth=4,
                )
            
            # visualize chosen grasp in object coordinatesystem
            visualize_grasp(
                self.visualizer,
                f"chosen_grasp_obj_frame",
                grasps_above_and_toward_table[0],
                color=[0, 0, 0],
                gripper_name=self.grasp_cfg.data.gripper_name,
                linewidth=100,
            )
            # print("X axis:", grasps_above_and_toward_table[0][:3, 0])
            # print("Y axis:", grasps_above_and_toward_table[0][:3, 1])
            # print("Z axis:", grasps_above_and_toward_table[0][:3, 2])
        
            dist_filtered_grasps = collision_free_grasps[~dist_mask]
            for i, grasp in enumerate(
                dist_filtered_grasps[:10]
            ):
                visualize_grasp(
                    self.visualizer,
                    f"to_close_to_table_grasps/{i:03d}/grasp",
                    grasp,
                    color=[255, 0, 0],
                    gripper_name=self.grasp_cfg.data.gripper_name,
                    linewidth=40,
                )

            plane_filtered_grasps = collision_free_grasps[~plane_mask]
            for i, grasp in enumerate(
                plane_filtered_grasps[:10]
            ):
                visualize_grasp(
                    self.visualizer,
                    f"not_approaching_from_above_grasps/{i:03d}/grasp",
                    grasp,
                    color=[0, 0, 255],
                    gripper_name=self.grasp_cfg.data.gripper_name,
                    linewidth=40,
                )

            cam_filtered_grasps = collision_free_grasps[~cam_mask]
            for i, grasp in enumerate(
                cam_filtered_grasps[:10]
            ):
                visualize_grasp(
                    self.visualizer,
                    f"not_approaching_from_the_front_grasps/{i:03d}/grasp",
                    grasp,
                    color=[255, 0, 255],
                    gripper_name=self.grasp_cfg.data.gripper_name,
                    linewidth=40,
                )


            # Visualize chosen grasps
            visualize_grasp(
                    self.visualizer,
                    f"chosen_grasp_cam_frame",
                    # grasps_centered[collision_free_mask][chosed_idx],
                    sasha_grasps[0],
                    color=[0, 0, 0],
                    gripper_name=self.grasp_cfg.data.gripper_name,
                    linewidth=40,
                )
            
            # # do weird stuff as it seems somewhere withing grasping pipeline there is weird stuff
            # tmp = Pose()
            # # tangle the stuff so that it gets assigne correctly
            # tmp.orientation.x = grasp_poses[0].orientation.w
            # tmp.orientation.y = grasp_poses[0].orientation.x
            # tmp.orientation.z = grasp_poses[0].orientation.y
            # tmp.orientation.w = grasp_poses[0].orientation.z

            # grasp_poses[0].orientation = tmp.orientation 
            # print("Tangled pose is:")
            # print(grasp_poses[0])           



            # Fill action result
            result.pose_results = [grasp_poses[0]] # list(grasp_poses) # replace to see all grasps in test client
            result.class_confidences = [collision_free_scores[0]] # list(collision_free_scores) # replace to see all grasp-confidences in test client
            result.class_names = ['Unknown Object']
            result.success = True
            rospy.loginfo('GraspGen: Grasp generation succesful')
        except Exception as e:
            rospy.logerr(f"Error in GraspGenWrapper: {e}")
            result.success = False
            rospy.loginfo('GraspGen: Grasp generation failed')
        finally:
            self.server.set_succeeded(result)

        

if __name__ == "__main__":
    wrapper = GraspGenWrapper()
    rospy.spin()
