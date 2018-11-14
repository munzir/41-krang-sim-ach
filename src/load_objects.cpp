#include <Eigen/Eigen>
#include <dart/dart.hpp>
#include <memory>
//====================================================================
dart::dynamics::SkeletonPtr createFloor() {
  using dart::dynamics;

  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  //  body->setFrictionCoeff(1e16);

  // Give the body a shape
  double floor_width = 50;
  double floor_height = 0.05;
  std::shared_ptr<BoxShape> box(
      new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  auto shapeNode =
      body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
          box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

//====================================================================
// Calculating the axis angle representation of orientation from headingInit
// and qBaseInit: RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
Eigen::AngleAxisd& GetKrangBaseAngleAxis(const double& headingInit,
                                         const double& qBaseInit) {
  Eigen::Transform<double, 3, Eigen::Affine> baseTf;
  baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit, Eigen::Vector3d::UnitX()))
      .prerotate(
          Eigen::AngleAxisd(-M_PI / 2 + headingInit, Eigen::Vector3d::UnitY()))
      .prerotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
  return Eigen::AngleAxisd(baseTf.rotation());
}

//====================================================================
void SetKrangInitPos(const SimConfig& params,
                     dart::dynamics::SkeletonPtr krang) {
  Eigen::AngleAxisd aa;
  aa = GetKrangBaseAngleAxis(params.headingInit, params.qBaseInit);
  Eigen::Matrix<double, 25, 1> q;
  q << aa.angle() * aa.axis(), params.xyzInit, params.qLWheelInit,
      params.qRWheelInit, params.qWaistInit, params.qTorsoInit,
      params.qKinectInit, params.qLeftArmInit, params.qRightArmInit;
  krang->setPositions(q);
}

//====================================================================
dart::dynamics::SkeletonPtr createKrang(const SimConfig& params) {
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr krang;
  krang = loader.parseSkeleton(parse.krangUrdfPath);
  krang->setName("krang");

  /*ifstream file;
  char line[1024];
  file = ifstream(params.default);
  assert(file.is_open());
  file.getline(line, 1024);
  std::istringstream stream;
  stream = std::istringstream(line);
  size_t i;
  i = 0;
  Eigen::Matrix<double, 24, 1>
      initPoseParams;  // heading, qBase, x, y, z, qLWheel, qRWheel, qWaist,
                       // qTorso, qKinect, qLArm0, ... qLArm6, qRArm0, ...,
                       // qRArm6
  double newDouble;
  while ((i < 24) && (stream >> newDouble)) initPoseParams(i++) = newDouble;
  file.close();
  double headingInit, qBaseInit, qLWheelInit, qRWheelInit, qWaistInit,
      qTorsoInit, qKinectInit;
  Eigen::Vector3d xyzInit;
  Eigen::Matrix<double, 7, 1> qLeftArmInit;
  Eigen::Matrix<double, 7, 1> qRightArmInit;
  headingInit = params.initPose(0);
  qBaseInit = params.initPose(1);
  xyzInit << params.initPose.segment(2, 3);
  qLWheelInit = params.initPose(5);
  qRWheelInit = params.initPose(6);
  qWaistInit = params.initPose(7);
  qTorsoInit = params.initPose(8);
  qKinectInit = params.initPose(9);
  qLeftArmInit << params.initPose.segment(10, 7);
  qRightArmInit << params.initPose.segment(17, 7);*/

  // Set the positions
  SetKrangInitPos(params, krang);

  // If balanced init pose is required
  if (params.initWithBalancePose) {
    Eigen::Vector3d COM;
    COM = krang->getCOM() - params.xyzInit;
    double th = atan2(COM(0), COM(2));

    // Adjust qBaseInit to bring COM on top of wheels and set the positions
    // again
    params.qBaseInit -= th;
    SetKrangInitPos(params, krang);
  }

  // Set wheel frictions
  krang->getJoint(0)->setDampingCoefficient(0, 0.5);
  krang->getJoint(1)->setDampingCoefficient(0, 0.5);

  return krang;
}
