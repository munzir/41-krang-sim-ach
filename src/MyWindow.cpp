#include <dart/dart.hpp>

MyWindow::MyWindow(const dart::dynamics::WorldPtr& world) {
  // Attach the world passed in the input argument to the window, and fetch
  // the robot from the world
  setWorld(world);
}

