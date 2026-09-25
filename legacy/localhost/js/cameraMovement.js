// code copied from the delta robot example \/\/

function mouseWheel(event) {
  if (mouseX < canvas.width){
    display.cameraTarget = glm.add(display.cameraTarget, glm.mul(glm.normalize(glm.sub(display.cameraTarget, display.cameraPosition)), event.delta/100));
    display.cameraPosition = glm.add(display.cameraPosition, glm.mul(glm.normalize(glm.sub(display.cameraTarget, display.cameraPosition)), event.delta/100));
    return false;
  };
};

function mousePressed() {
  xOffset = mouseX;
  yOffset = mouseY;
}

function mouseClicked(){
    if((objectAtMouse() > 0) && (mouseButton === LEFT)){clickableObjectCallback[objectAtMouse()]();};
}

function mouseDragged() {
    if ((mouseX < canvas.width) && (mouseButton == CENTER)){ //rotating
      var yRotationAxis = glm.normalize(glm.cross(glm.sub(display.cameraTarget, display.cameraPosition),display.cameraUp));
      
      display.cameraPosition =  glm.add((glm.angleAxis((-3 * (mouseX - xOffset) / width), display.cameraUp))['*'](glm.sub(display.cameraPosition, display.cameraTarget)), display.cameraTarget);
      
      display.cameraPosition =  glm.add((glm.angleAxis((3 * (mouseY - yOffset) / height), yRotationAxis))['*'](glm.sub(display.cameraPosition, display.cameraTarget)), display.cameraTarget);
      display.cameraUp = (glm.angleAxis((3 * (mouseY - yOffset) / height), yRotationAxis))['*'](display.cameraUp);
      
      xOffset = mouseX;
      yOffset = mouseY;
    }

    if ((mouseX < canvas.width) && (mouseButton == LEFT)){ //panning
      var yRotationAxis = glm.normalize(glm.cross(glm.sub(display.cameraTarget, display.cameraPosition),display.cameraUp));
      
      display.cameraTarget = glm.add(display.cameraTarget, glm.mul(display.cameraUp,(-10 * (mouseY - yOffset) / height)));
      display.cameraPosition = glm.add(display.cameraPosition, glm.mul(display.cameraUp,(-10 * (mouseY - yOffset) / height)));
      
      display.cameraTarget = glm.add(display.cameraTarget, glm.mul(glm.normalize(yRotationAxis),-10 * ((mouseX - xOffset) / width)));
      display.cameraPosition = glm.add(display.cameraPosition, glm.mul(glm.normalize(yRotationAxis),-10 * ((mouseX - xOffset) / width)));
      
      xOffset = mouseX;
      yOffset = mouseY;
    }
  return false;
}
