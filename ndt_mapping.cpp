#

void ndt_mapping::initialization(){
  previous_pose = Eigen::Matrix4f::Identity();
  current_pose = Eigen::Matrix4f::Identity();
  guess_pose = Eigen::Matrix4f::Identity();
  added_matching_pose = Eigen::Matrix4f::Identity();
  delta_pose = Eigen::Matrix4f::Identity();
  guess_pose_odom = Eigen::Matrix4f::Identity();
  
  current_velocity_x = 0;
  
}

void ndt_mapping::odomCalculate(ros::Time current_time){
  transform_.MsgToEigen(odom.pose.pose, guess_pose_odom);
  current_time -= ros::Duration(predicted_lag);
  if(enable_syn_flag){
    if(odom_deq.size() < 1){
      guess_pose_odom(0, 3) = 0;
      guess_pose_odom(1, 3) = 0;
    }
    else{
      int get_i = 0;
      int temp_i = odom_deq.size();
      
      
      
void ndt_mapping::registration(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_points,
                               Eigen::Matrix4f &input_guess, Eigen::Matrix4f &output){
  if(0 == odom_or_matching){
    double temp_dist2 = sqrt(pow(input_guess(0, 3) - added_matching_pose(0, 3), 2) +
                             pow(input_guess(1, 3) - added_matching_pose(1, 3), 2));
    if(temp_dist2 >= matching add_step_length){
      addVertex(input_guess);
      addEdge(vertex_ID -1, added_odom_pose, vertex_ID, input_guess);
      addTrajectoryNode(input_points, input_guess);
      ndt.setInputSource(input_points);
      if(vertex_ID %30 == 0){
        odom_edge_flag = true;
        addEdge(1, trajectory_[0].pose, vertex_ID, input_guess);
        odom_edge_flag = false;
      }
      
      int temp_closure_ID;
      added_odom_pose = input_guess;
      enable_matching_flag = true;
      if(isLoopClosure(vertex_ID, input_guess, temp_closure_ID)) {
        calculateLoopClosure(temp_closure_ID, input_guess, output);
        addEdge(temp_closure_ID, trajectory_[temp_closure_ID - 1].pose,  vertex_ID,
                trajectory_[temp_closure_ID - 1].pose *output);
        is_loop_closure = false;
      }
    }else{
      output = input_guess;
      return;
    }
  }
  ndt_registration(input_guess, output);
  
  void ndt_mapping::getOutput(Eigen::Matrix4f input1, Eigen::Matrix4f input2 
                              pcl::PointCloud<pcl::PointXYZI>::Ptr& input_points,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr& output_points){
    tf::Matrix3x3 mat_limit;
    transform_.EigenMatrixToTFMatrix(input1, mat_limit);
    double temp_roll, temp_pitch, temp_yaw;
    mat_limit.getRPY(
        
        
      
    
    
