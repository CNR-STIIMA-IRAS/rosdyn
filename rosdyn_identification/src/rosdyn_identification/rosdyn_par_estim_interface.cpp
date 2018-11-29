
#include <pwd.h>
#include <unistd.h>
#include <sys/types.h>

#include <cmath>
#include <functional>   // std::minus
#include <numeric>      // std::accumulate
#include <dirent.h>
#include <random>
#include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.h> 

#include <rosdyn_identification/rosdyn_par_estim_interface.h>
//#include<Eigen/StdVector>

PLUGINLIB_EXPORT_CLASS(rosdyn::MetoParEstimInterfaceNodelet, nodelet::Nodelet) 

namespace rosdyn
{
  
  inline int getdir (const std::string& dir, std::vector<std::string>& files)
  {
    DIR *dp;
    struct dirent *dirp;
    
    if((dp  = opendir(dir.c_str())) == NULL) 
    {
      std::cout << "Error(" << errno << ") opening " << dir << std::endl;
      return errno;
    }
    
    while ((dirp = readdir(dp)) != NULL) 
      files.push_back( std::string(dirp->d_name) );    
    
    closedir(dp);
    return 0;
  }
  
  void MetoParEstimInterfaceNodelet::onInit()
  {
    std::vector<std::string> args = getMyArgv();
  
    m_stop = false;
    
    m_meto_par_estim_as.reset(new actionlib::SimpleActionServer<rosdyn_identification_msgs::MetoParEstimAction>( getNodeHandle(), "meto_param_estimation", 
                              boost::bind( &MetoParEstimInterfaceNodelet::metoParEstimCB,  this, _1 ), false   ));
    
    m_meto_par_estim_as->start();
    
    m_main_thread  = std::thread(&rosdyn::MetoParEstimInterfaceNodelet::main, this);
    
  };


  MetoParEstimInterfaceNodelet::~MetoParEstimInterfaceNodelet()
  {
    m_stop = true;
    if (m_main_thread.joinable())
      m_main_thread.join(); 
  };

  
  void MetoParEstimInterfaceNodelet::metoParEstimCB( const rosdyn_identification_msgs::MetoParEstimGoalConstPtr& goal )
  { 
    
    std::string trj_name_           = goal->trj_namespace;
    
//     std::string add_info_namespace_ = goal->add_info_namespace;
//     std::string add_info_name_      = goal->add_info_name;
//     std::string add_info_path_      = goal->add_info_path;

    rosdyn_identification_msgs::MetoParEstimResult m_result_;
    rosdyn_identification_msgs::MetoParEstimFeedback m_feedback_;
    
    bool verbose_ = true;
    if ( !getNodeHandle().getParam( std::string(std::string(m_namespace)+"/verbose").c_str(), verbose_) )
    {
      ROS_DEBUG("Impossible to find %s/verbose, set false\n", m_namespace.c_str() );
      verbose_=false;
    }
    
    std::string topic_type_ = "JointState";
    std::string log_topic_name_ = goal->trj_namespace;
    
    std::string topic_name_tmp_ = log_topic_name_;
    std::replace( topic_name_tmp_.begin(), topic_name_tmp_.end(), '/', '_');
        
    std::string robot_description;
    if (!getNodeHandle().getParam("robot_description", robot_description) )
    {
      ROS_ERROR("Impossible to find robot_description\n");
      m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::GENERIC_ERROR;
      m_meto_par_estim_as->setAborted(m_result_);
      return;
    }
    
    std::vector<std::string> controller_joint_names_;
    if ( !getNodeHandle().getParam(std::string(m_namespace+"/controller_joint_names"), controller_joint_names_) )
    {
      ROS_ERROR("Impossible to find %s\n", std::string(m_namespace+"/controller_joint_names").c_str() );
      m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::GENERIC_ERROR;
      m_meto_par_estim_as->setAborted(m_result_);
      return;
    }
    
    rosdyn::MetoParEstim metoParEstim( getNodeHandle(), 
                                           robot_description);
    
    
    std::vector<std::string> joints_name = metoParEstim.getRobotJointName();
    unsigned int number_of_joint_from_xml_ = joints_name.size();
    
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string test_path_( homedir + std::string("/.ros") );
    
    // Find all the files that need to be loaded
    std::string dir = test_path_ + "/";
    std::vector<std::string> files = std::vector<std::string>();

    if (getdir( dir, files ))
    {
      ROS_ERROR("No valid file to be loaded %s\n", test_path_.c_str() );
      m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::GENERIC_ERROR;
      m_meto_par_estim_as->setAborted(m_result_);
      return;
    }

    std::vector<std::string> file_to_be_loaded_;
    for ( auto singleFile : files ) 
    {
      std::size_t pos_file_       = singleFile.find( trj_name_ );
      std::size_t pos_topic_      = singleFile.find( topic_name_tmp_ );
      std::size_t pos_is_fake_    = singleFile.find( "fake_controller" );    
      std::size_t pos_extension_  = singleFile.find( ".bin" );   
      
      if( (pos_file_ != std::string::npos) && 
          ( pos_topic_ != std::string::npos ) &&
          ( pos_is_fake_ == std::string::npos) && 
          ( pos_extension_ == singleFile.size()-4 ) )
        file_to_be_loaded_.push_back( singleFile.substr( pos_file_ ) );
    }
    
    if ( file_to_be_loaded_.size() == 0 )
    {
      ROS_ERROR("No valid file to be loaded %s\n", test_path_.c_str() );
      m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::GENERIC_ERROR;
      m_meto_par_estim_as->setAborted(m_result_);
      return;
    }
    
    
    std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd>> vctFileData;
    
    for ( const std::string& fileToLoad : file_to_be_loaded_ )
    {
      ROS_INFO("Loading a new binary file, please wait a moment...");
    
      m_file_name = test_path_ + "/" + fileToLoad;
      
      std::ifstream is;
      is.open (m_file_name, std::ios::in | std::ios::binary );
      
      // get length of file:
      is.seekg ( 0, std::ios::end );
      unsigned int length = is.tellg(); 
      is.seekg ( 0, std::ios::beg );
      
      // read data as a block:
      int number_of_sample = length / sizeof(double) / ( 1+3*number_of_joint_from_xml_ );
      
      Eigen::MatrixXd singleFileData( (1+3*number_of_joint_from_xml_), number_of_sample );
      singleFileData.setZero();
      
      if( length != ( singleFileData.rows() * singleFileData.cols() * sizeof(double) ) )
      {
        ROS_ERROR("Dimensions mismatch between the number of bytes in the binary file and the reconstructed matrix! (file name: %s)",fileToLoad.c_str());
        ROS_ERROR("length: %d", length);
        ROS_ERROR("rows(): %zu", singleFileData.rows());
        ROS_ERROR("cols(): %zu", singleFileData.cols());
        ROS_ERROR("sizeof(double): %zu", sizeof(double));
        
        m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::GENERIC_ERROR;
        m_meto_par_estim_as->setAborted(m_result_);
        return;
      }  
      
      is.read ( (char*)singleFileData.data(), length );
      is.close();
      
      vctFileData.push_back( singleFileData );
      
      ROS_INFO("File: %s  loaded!", fileToLoad.c_str() );  
    }
    
    int nCols = 0;
    int nRows = vctFileData.front().rows();
    for ( auto singFile : vctFileData )
    {
      nCols += singFile.cols();
      if ( nRows != singFile.rows() )
      {
        ROS_ERROR("Dimensions mismatch: data extracted from two different files have a different number of rows.");
        m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::GENERIC_ERROR;
        m_meto_par_estim_as->setAborted(m_result_);
        return;
      }
    }
    
    
  
    Eigen::MatrixXd qf_full( nCols, number_of_joint_from_xml_ ), 
                    Dqf_full( nCols, number_of_joint_from_xml_ ), 
                    DDqf_full( nCols, number_of_joint_from_xml_ ), 
                    efff_full( nCols, number_of_joint_from_xml_ );

    qf_full.setZero(); Dqf_full.setZero(); DDqf_full.setZero(); efff_full.setZero();
    
    ROS_INFO("Starting to compute dynamics parameters...");
    
    int st_row_ = 0;
    int size_row_ = 0;
    for ( auto singFile : vctFileData )
    {        
      Eigen::MatrixXd fileData = singFile;
      
      if (verbose_)
        ROS_INFO("Loaded new data file with %zu rows and %zu columns", fileData.rows(), fileData.cols() );
      
      Eigen::MatrixXd q( singFile.cols(), number_of_joint_from_xml_ ),
                      Dq( singFile.cols(), number_of_joint_from_xml_ ), 
                      DDq( singFile.cols(), number_of_joint_from_xml_ ), 
                      eff( singFile.cols(), number_of_joint_from_xml_ );
      
      Eigen::MatrixXd qf( singFile.cols(), number_of_joint_from_xml_ ), 
                      Dqf( singFile.cols(), number_of_joint_from_xml_ ), 
                      DDqf( singFile.cols(), number_of_joint_from_xml_ ), 
                      efff( singFile.cols(), number_of_joint_from_xml_ );                
                      
      q.setZero(); Dq.setZero(); DDq.setZero(); eff.setZero();      
      qf.setZero(); Dqf.setZero(); DDqf.setZero(); efff.setZero();
      
      for ( unsigned int idxJnt=0; idxJnt<number_of_joint_from_xml_; idxJnt++ )
      {      
        q.col(idxJnt) = fileData.row(   idxJnt + 1 ).transpose();
        Dq.col(idxJnt) = fileData.row(  idxJnt + 1 + number_of_joint_from_xml_  ).transpose();
        eff.col(idxJnt) = fileData.row( idxJnt + 1 + number_of_joint_from_xml_ * 2 ).transpose();
                
        eigen_control_toolbox::FirstOrderLowPass filt_pos;   
        filt_pos.importMatricesFromParam(      getNodeHandle(),m_namespace + "/filter");
        filt_pos.setStateFromLastIO(      q.col(idxJnt).head(1),   q.col(idxJnt).head(1));

        eigen_control_toolbox::FirstOrderLowPass filt_vel;
        filt_vel.importMatricesFromParam(      getNodeHandle(),m_namespace + "/filter");
        filt_vel.setStateFromLastIO(     Dq.col(idxJnt).head(1),  Dq.col(idxJnt).head(1));
     
        Eigen::VectorXd init_acc(1);init_acc.setZero();
        eigen_control_toolbox::FirstOrderLowPass filt_acc;
        filt_acc.importMatricesFromParam(      getNodeHandle(),m_namespace + "/filter");
        filt_acc.setStateFromLastIO(    init_acc,                Dq.col(idxJnt).head(1));
        
        eigen_control_toolbox::FirstOrderLowPass filt_eff;
        filt_eff.setStateFromLastIO( eff.col(idxJnt).head(1), eff.col(idxJnt).head(1));
        filt_eff.importMatricesFromParam(      getNodeHandle(),m_namespace + "/filter");
        filt_eff.setStateFromLastIO( eff.col(idxJnt).head(1), eff.col(idxJnt).head(1));
        
        for (unsigned int iStep=0; iStep<q.rows(); iStep++)
        {
          qf(iStep, idxJnt)     = filt_pos.update(q(iStep, idxJnt));
          Dqf(iStep, idxJnt)    = filt_vel.update(Dq(iStep, idxJnt));
          DDqf(iStep, idxJnt)   = filt_acc.update(Dq(iStep, idxJnt));
          efff(iStep, idxJnt)   = filt_eff.update(eff(iStep, idxJnt));
        }
      }
      
      
      size_row_ = qf.rows();
      
      if ( verbose_ )
        ROS_INFO("Writing %d rows and %zu columns, from row = %d and col = %d", size_row_, qf.cols(), st_row_, 0 );
      
      
      qf_full.block( st_row_,0,size_row_,qf.cols() )      = qf;
      Dqf_full.block( st_row_,0,size_row_,Dqf.cols() )    = Dqf;
      DDqf_full.block( st_row_,0,size_row_,DDqf.cols() )  = DDqf;
      efff_full.block( st_row_,0,size_row_,efff.cols() )  = efff;
      
      st_row_ += qf.rows();
    }
    
    std::mt19937 eng{ std::random_device{}() };
    Eigen::VectorXi indices = Eigen::VectorXi::LinSpaced(qf_full.rows(), 0, qf_full.rows());
    
    
    std::shuffle(indices.data(), indices.data() + qf_full.rows(),eng);
    
    qf_full= indices.asPermutation() * qf_full;
    Dqf_full= indices.asPermutation() * Dqf_full;
    DDqf_full= indices.asPermutation() * DDqf_full;
    efff_full= indices.asPermutation() * efff_full;
    
    unsigned int identification_part = std::min((unsigned int)1e5,((unsigned int)qf_full.rows())/2);
    
    
    // IDENTIFICATION 
    Eigen::MatrixXd qf_ident   = qf_full  .topRows(identification_part);
    Eigen::MatrixXd Dqf_ident  = Dqf_full .topRows(identification_part);
    Eigen::MatrixXd DDqf_ident = DDqf_full.topRows(identification_part);
    Eigen::MatrixXd efff_ident = efff_full.topRows(identification_part);
    
    ROS_DEBUG("Waits for parameters estimation...");
    Eigen::MatrixXd PhiR  = metoParEstim.getTrajectoryRegressor(qf_ident, Dqf_ident, DDqf_ident);
    Eigen::VectorXd PiR   = metoParEstim.getEstimatedParameters(efff_ident);
    
    ROS_DEBUG_STREAM("Parameters:\n"<<PiR);
    
    Eigen::MatrixXd Phi  = metoParEstim.getTrajectoryFullRegressor(qf_ident, Dqf_ident, DDqf_ident);
    Eigen::VectorXd Pi    = metoParEstim.getFullEstimatedParameters();
    ROS_DEBUG_STREAM("Full Parameters:\n"<<Pi);
    Eigen::VectorXd Tau_ident   = PhiR * PiR;
    Eigen::VectorXd Tau_ident_full   = Phi * Pi;
    ROS_DEBUG("error full base = %f",(Tau_ident-Tau_ident_full).norm());
    
    Eigen::Map<Eigen::MatrixXd> map_tau_ident(Tau_ident.data(), efff_ident.cols(),efff_ident.rows());
    Eigen::MatrixXd efff_ident_model=map_tau_ident.transpose();
    
    
    // VALIDATION
    Eigen::MatrixXd qf_vali    = qf_full.  bottomRows(qf_full.rows()-identification_part);
    Eigen::MatrixXd Dqf_vali   = Dqf_full. bottomRows(qf_full.rows()-identification_part);
    Eigen::MatrixXd DDqf_vali  = DDqf_full.bottomRows(qf_full.rows()-identification_part);
    Eigen::MatrixXd efff_vali  = efff_full.bottomRows(qf_full.rows()-identification_part);
    
    Eigen::MatrixXd PhiR_vali  = metoParEstim.getTrajectoryRegressor(qf_vali, Dqf_vali, DDqf_vali);
    Eigen::VectorXd Tau_vali   = PhiR_vali * PiR;
    
    Eigen::Map<Eigen::MatrixXd> map_tau_vali(Tau_vali.data(), efff_vali.cols(),efff_vali.rows());
    Eigen::MatrixXd efff_vali_model=map_tau_vali.transpose();
    
    rosdyn_identification_msgs::TorqueFitting ident;
    rosdyn_identification_msgs::TorqueFitting vali;
    
    for ( std::size_t iJnt=0; iJnt<number_of_joint_from_xml_; iJnt++ )    
    {
      Eigen::VectorXd error= (efff_ident-efff_ident_model).col(iJnt);
      double rmse=eigen_utils::standard_deviation( error );
      double corr=eigen_utils::correlation( efff_ident.col(iJnt),efff_ident_model.col(iJnt) );
      ident.joint_names.push_back(controller_joint_names_.at(iJnt));
      ident.rms_error.push_back(rmse);
      ident.real_model_correlation.push_back(corr);
      
      error= (efff_vali-efff_vali_model).col(iJnt);
      rmse=eigen_utils::standard_deviation( error );
      corr=eigen_utils::correlation( efff_vali.col(iJnt), efff_vali_model.col(iJnt) );
      
      vali.joint_names.push_back(controller_joint_names_.at(iJnt));
      vali.rms_error.push_back(rmse);
      vali.real_model_correlation.push_back(corr);
    }
    m_result_.identification=ident;
    m_result_.validation=vali;
    
    m_meto_par_estim_as->publishFeedback(m_feedback_);
   ros::Duration(0.5).sleep();
   /* 
    if ( !metoParEstim.saveParXml(std::string(xml_path_+"/"+xml_file_name_), 
                                  std::string(add_info_path_+"/"+add_info_name_),
                                  add_info_namespace_,
                                  Pi,
                                  controller_joint_names_ ) )
    {
      ROS_ERROR("Impossible to save the estimated parametrs into XML file.");
      m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::GENERIC_ERROR;
      m_meto_par_estim_as->setAborted(m_result_);
      return;
    }
    */
   
    ROS_INFO("Estimation complete!");
    m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::SUCCESSFUL;
    m_meto_par_estim_as->setSucceeded(m_result_);
    return;
  }


  void MetoParEstimInterfaceNodelet::main()
  {
    ROS_INFO("Starting THREAD MetoParEstimInterfaceNodelet");
     
    m_namespace = "meto_cfg";
    
    while( (ros::ok()) && (!m_stop) )
    {
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
   
    ROS_INFO("End of MetoParEstimInterfaceNodelet");
    return;
  };


}
