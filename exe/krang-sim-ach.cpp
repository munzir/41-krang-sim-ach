int main(int argc, char* argv[]) {
  // Create world
  WorldPtr world = std::make_shared<World>();

  // Load Floor
  SkeletonPtr floor = createFloor();
  world->addSkeleton(floor);  // add ground and robot to the world pointer

  // Load robot
  SkeletonPtr robot = createKrang();
  world->addSkeleton(robot);

  // Create window
  MyWindow window(world);

  // Run the world
  glutInit(&argc, argv);
  window.initWindow(1280, 720, "Krang Simulation with Ach");
  glutMainLoop();

  return 0;
}
