#include <TFT_eSPI.h>
#include "player.h"
#include "enemy.h"


TFT_eSPI tft = TFT_eSPI();
TFT_eSprite img = TFT_eSprite(&tft);
int maxSpriteX = 230;
int maxSpriteY = 230;

struct myObject {
  float x, y;      // Position
  float vx, vy;    // Velocity
  float radius;
  float mass;
  int color;       // Color variable
  int texture;
};

const int maxObjects = 100;  // Maximum number of objects
myObject objects[maxObjects]; // Array to store objects
int objSizeMin = 4;
int objSizeMax = 8;
int objTextureMin = 0;
int objTextureMax = 1;

float dampingFactor = 1;  // Adjust as needed for damping effect
const float dampingFactorD = 0.90;  // Adjust as needed for damping effect
float mygravity = 0.0;               // Initial gravity (turned off)
float gravityStartTime = 5000;     // Time (in milliseconds) to turn on gravity
unsigned long gravityChangeInterval = 5000; // Time interval to change gravity direction (in milliseconds)
unsigned long lastGravityChangeTime = 0;  // Time of the last gravity change
bool gravityEnabled = false;
unsigned long startTime = 0;       // Variable to store the start time
float gravityG = 0.1;
float boundaryRadius = 115.0;      // Initial boundary radius
//float boundaryRadius = 100.0;      // Initial boundary radius
int backgroundColor = TFT_BLACK;


// Define Obstacle structure
struct Obstacle {
  float x, y;      // Position
  float width, height;
  float vx, vy;    // Velocity
  float minBoundX, maxBoundX; // Minimum and maximum bounds for motion
  float minBoundY, maxBoundY; // Minimum and maximum bounds for motion
};
const int maxObstacles = 3;  // Maximum number of obstacles
Obstacle obstacles[maxObstacles];  // Array to store obstacles

void resizeAndRotateImage(const unsigned short* imageData, int width, int height, float scaleFactor, float rotationDegrees, unsigned short* outputBuffer, int& newWidth, int& newHeight) {
  // Calculate the new dimensions after scaling
  newWidth = static_cast<int>(width * scaleFactor);
  newHeight = static_cast<int>(height * scaleFactor);

    // Convert degrees to radians
  float rotationAngle = rotationDegrees * (PI / 180.0);

  // Calculate the center of the original image
  float centerX = width / 2.0;
  float centerY = height / 2.0;

  // Perform the resizing and rotation
  for (int y = 0; y < newHeight; y++) {
    for (int x = 0; x < newWidth; x++) {
      // Calculate the corresponding pixel from the original image
      float originalX = (x - newWidth / 2.0) / scaleFactor;
      float originalY = (y - newHeight / 2.0) / scaleFactor;

      // Apply rotation around the image center
      float rotatedX = centerX + (originalX * cos(rotationAngle) - originalY * sin(rotationAngle));
      float rotatedY = centerY + (originalX * sin(rotationAngle) + originalY * cos(rotationAngle));

      // Ensure the coordinates are within bounds
      if (rotatedX >= 0 && rotatedX < width && rotatedY >= 0 && rotatedY < height) {
        int sourceX = static_cast<int>(rotatedX);
        int sourceY = static_cast<int>(rotatedY);
        outputBuffer[y * newWidth + x] = imageData[sourceY * width + sourceX];
      }
      else {
        // Handle out-of-bounds pixels (e.g., set to a background color)
        outputBuffer[y * newWidth + x] = 0x0000; // Set to black, adjust as needed
      }
    }
  }
}




void resolveCollision(myObject &obj1, myObject &obj2) {
  // Calculate the relative velocity between the two objects.
  float dx = obj2.x - obj1.x;
  float dy = obj2.y - obj1.y;
  float distance = sqrt(dx * dx + dy * dy);
  float normalX = dx / distance;
  float normalY = dy / distance;

  float relativeVelocityX = obj2.vx - obj1.vx;
  float relativeVelocityY = obj2.vy - obj1.vy;
  float dotProduct = relativeVelocityX * normalX + relativeVelocityY * normalY;

  // Check if the objects are moving towards each other (dotProduct < 0).
  if (dotProduct < 0) {
    // Calculate the impulse (change in momentum) to resolve the collision.
    float impulse = (2.0 * dotProduct) / (obj1.mass + obj2.mass);

    // Update the velocities of both objects based on the impulse and mass.
    obj1.vx += impulse * obj2.mass * normalX;
    obj1.vy += impulse * obj2.mass * normalY;
    obj2.vx -= impulse * obj1.mass * normalX;
    obj2.vy -= impulse * obj1.mass * normalY;

    // Move the objects apart to prevent overlap.
    float overlap = (obj1.radius + obj2.radius - distance) / 2.0;
    obj1.x -= overlap * normalX;
    obj1.y -= overlap * normalY;
    obj2.x += overlap * normalX;
    obj2.y += overlap * normalY;
    
  }
}

void circularBoundary(myObject &obj, int WIDTH, int HEIGHT) {
  // Calculate the distance from the object's center to the center of the circular boundary.
  float distanceToCenter = sqrt(pow(obj.x - WIDTH / 2, 2) + pow(obj.y - HEIGHT / 2, 2));
  
  // Check if any part of the object is outside the circular boundary.
  if (distanceToCenter + obj.radius > boundaryRadius) {
    // Calculate the angle between the object's center and the center of the circular boundary.
    float angle = atan2(obj.y - HEIGHT / 2, obj.x - WIDTH / 2);
    
    // Move the object's center to the boundary while keeping its radius in mind.
    obj.x = WIDTH / 2 + (boundaryRadius - obj.radius) * cos(angle);
    obj.y = HEIGHT / 2 + (boundaryRadius - obj.radius) * sin(angle);
    
    // Reflect the object's velocity vector off the circular boundary.
    float dotProduct = obj.vx * cos(angle) + obj.vy * sin(angle);
    obj.vx -= 2 * dotProduct * cos(angle);
    obj.vy -= 2 * dotProduct * sin(angle);
  }
}

void boundries(myObject &obj, int WIDTH, int HEIGHT) {
  // Check if the object hits the left or right boundary.
  if (obj.x - obj.radius < 0) {
    obj.x = obj.radius;
    obj.vx *= -1; // Reverse the velocity to bounce off the wall.
  } else if (obj.x + obj.radius > WIDTH) {
    obj.x = WIDTH - obj.radius;
    obj.vx *= -1; // Reverse the velocity to bounce off the wall.
  }

  // Check if the object hits the top or bottom boundary.
  if (obj.y - obj.radius < 0) {
    obj.y = obj.radius;
    obj.vy *= -1; // Reverse the velocity to bounce off the wall.
  } else if (obj.y + obj.radius > HEIGHT) {
    obj.y = HEIGHT - obj.radius;
    obj.vy *= -1; // Reverse the velocity to bounce off the wall.
  }
}
void applyGravity(myObject &obj) {
  obj.vy += mygravity; // Apply gravity force to the y-velocity
}
void resolveCollisionWithObstacle(myObject &obj, Obstacle &obstacle) {
  float objLeft = obj.x - obj.radius;
  float objRight = obj.x + obj.radius;
  float objTop = obj.y - obj.radius;
  float objBottom = obj.y + obj.radius;

  float obstacleLeft = obstacle.x;
  float obstacleRight = obstacle.x + obstacle.width;
  float obstacleTop = obstacle.y;
  float obstacleBottom = obstacle.y + obstacle.height;

  // Check for overlap in the x and y axes
  if (abs(objRight - obstacleLeft) < 3) {
    // Left side collision
    obj.x = obstacleLeft - obj.radius;
    obj.vx = -abs(obj.vx) + (obstacle.vx * 0.5);
  } else if (abs(objLeft - obstacleRight) < 3) {
    // Right side collision
    obj.x = obstacleRight + obj.radius;
    obj.vx = abs(obj.vx) + (obstacle.vx * 0.5);
  } else if (abs(objBottom - obstacleTop) < 3) {
    // Top collision
    obj.y = obstacleTop - obj.radius;
    obj.vy = -abs(obj.vy) + (obstacle.vy * 0.5);
  } else if (abs(objTop - obstacleBottom) < 3) {
    // Bottom collision
    obj.y = obstacleBottom + obj.radius;
    obj.vy = abs(obj.vy) + (obstacle.vy * 0.5);
  }

  // Apply a damping factor to reduce the object's velocity upon collision
  obj.vx *= 0.9;
  obj.vy *= 0.9;
}

void xresolveCollisionWithObstacle(myObject &obj, Obstacle &obstacle) {

  float objLeft = obj.x - obj.radius;
  float objRight = obj.x + obj.radius;
  float objTop = obj.y - obj.radius;
  float objBottom = obj.y + obj.radius;

  float obstacleLeft = obstacle.x;
  float obstacleRight = obstacle.x + obstacle.width;
  float obstacleTop = obstacle.y;
  float obstacleBottom = obstacle.y + obstacle.height;

  // Check for overlap in the x and y axes
  if (abs(objRight - obstacleLeft) < 3){
    obj.x = obstacleLeft - obj.radius;  // Adjust the x position to the left of the obstacle
    if(obstacle.vx < 0){
      obj.vx = -abs(obj.vx)+(obstacle.vx*0.5);
    }else{
      obj.vx = abs(obj.vx)+(obstacle.vx*0.5);
    }
    return;
    }
  if(abs(objLeft - obstacleRight) < 3) {
    obj.x = obstacleRight + obj.radius;  // Adjust the x position to the right of the obstacle
    if(obstacle.vx < 0){
      obj.vx = -abs(obj.vx)+(obstacle.vx*0.5);
    }else{
      obj.vx = abs(obj.vx)+(obstacle.vx*0.5);
    }
    return;
    }

  // Reverse object's velocity when colliding with a side of obstacle
  obj.vx = -obj.vx;
  obj.vy = -obj.vy;

  // Apply a damping factor to reduce the object's velocity upon collision
  obj.vx *= 0.9; // Adjust the damping factor as needed
  obj.vy *= 0.9;
}
bool checkCollisionWithObstacle(myObject &obj, Obstacle &obstacle) {
  // Check if the object collides with the obstacle
  // You can use bounding box collision detection or more complex shapes if needed
  float objLeft = obj.x - obj.radius;
  float objRight = obj.x + obj.radius;
  float objTop = obj.y - obj.radius;
  float objBottom = obj.y + obj.radius;

  float obstacleLeft = obstacle.x;
  float obstacleRight = obstacle.x + obstacle.width;
  float obstacleTop = obstacle.y;
  float obstacleBottom = obstacle.y + obstacle.height;

  // Check for overlap in the x and y axes
  if (objRight >= obstacleLeft && objLeft <= obstacleRight && objBottom >= obstacleTop && objTop <= obstacleBottom) {
    return true; // Collision detected
  }
    return false; // No collision
}
void updateObstacleMotion(Obstacle &obstacle) {
  // Update the obstacle's x and y positions based on their velocities
//  obstacle.x += obstacle.vx;
//  obstacle.y += obstacle.vy;
  if(obstacle.vx != 0){
    // Check and reverse the motion direction when reaching bounds
    if (obstacle.x < obstacle.minBoundX || obstacle.x > obstacle.maxBoundX) {
      obstacle.vx = -obstacle.vx;
      }
    }
  if(obstacle.vy != 0){
    // Check and reverse the motion direction when reaching bounds
    if (obstacle.y < obstacle.minBoundY || obstacle.y > obstacle.maxBoundY) {
      obstacle.vy = -obstacle.vy;
      }
    }
}
int bastekCoord[4]={80, 100, 130, 145};
void makeObstacleBasket(int x1, int y1, int x2, int y2, float vx, float vy, int swingX, int swingY){
    // make a bucket

  obstacles[0] = {x1, y1, 5,    y2-y1+5    , vx , vy  , x1-swingX       , x1+swingX      , y1-swingY       ,y1+swingY       };  // Example obstacle 1
  obstacles[1] = {x1, y2, x2-x1+5,    5    , vx , vy  , x1-swingX       , x1+swingX      , y2-swingY       ,y2+swingY       };  // Example obstacle 2
  obstacles[2] = {x2, y1, 5,    y2-y1+5    , vx , vy  , x2-swingX       , x2+swingX      , y1-swingY       ,y1+swingY       };  // Example obstacle 2
}



void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setSwapBytes(true);
  img.createSprite(maxSpriteX, maxSpriteY);
  img.setSwapBytes(true);
  tft.fillScreen(backgroundColor);

  // Initialize your objects dynamically with random colors and positions.
  for (int i = 0; i < maxObjects; i++) {
    bool overlap;
    do {
      overlap = false; // Initialize overlap flag to false
      objects[i].x = random(maxSpriteX);
      objects[i].y = random(maxSpriteY);
      
      // Check if the new object overlaps with existing objects
      for (int j = 0; j < i; j++) {
        float dx = objects[i].x - objects[j].x;
        float dy = objects[i].y - objects[j].y;
        float minDistance = objects[i].radius + objects[j].radius;
        if (sqrt(dx * dx + dy * dy) < minDistance) {
          overlap = true; // Set overlap flag to true
          break; // Exit the loop if overlap is detected
        }
      }
    } while (overlap); // Repeat the loop if overlap is detected
    
    // Once a non-overlapping position is found, proceed to set other properties
    objects[i].vx = random(-1, 1);
    objects[i].vy = random(-1, 1);
    objects[i].radius = random(objSizeMin, objSizeMax);
    objects[i].mass = objects[i].radius; // Simplified mass calculation
    objects[i].color = random(0xFFFF);  // Random color (16-bit RGB565 format)
    objects[i].texture = random(objTextureMin,objTextureMax);
  }

  // Initialize your obstacles with initial properties
  //                x  , y  , width, height, vx  , vy , minBoundX, maxBoundX, minBoundY, maxBoundY;
//  obstacles[0] = {050, 070 , 20   , 20    , 0   , 0.5, 30       , 100      , 70       ,150       };  // Example obstacle 1
//  obstacles[1] = {120, 100, 30   , 30    , 0.5 , 0  , 80       , 160      , 80       ,160       };  // Example obstacle 2
  // Initialize other objects and properties
  if(maxObstacles>=3){
    makeObstacleBasket(bastekCoord[0],bastekCoord[1],bastekCoord[2],bastekCoord[3], 1, 0, 30, 30);
  }
/*
  obstacles[0] = {80, 100, 5,    50    , 0.5   , 0, 50       , 100      , 70       ,150       };  // Example obstacle 1
  obstacles[1] = {80, 145, 55,    5    , 0.5 , 0  , 50       , 100      , 80       ,160       };  // Example obstacle 2
  obstacles[2] = {130, 100, 5,    50    , 0.5 , 0  , 100       , 150      , 80       ,160       };  // Example obstacle 2
*/
    // Store the current time.
    startTime = millis();
}
int countBallsinBasket(){
  int buf = 0;
  int counter = 0;
    for (int i = 0; i < maxObjects; i++) {
      if(objects[i].x > bastekCoord[0]+buf &&
         objects[i].x < bastekCoord[2]-buf &&
         objects[i].y > bastekCoord[1]+buf &&
         objects[i].y < bastekCoord[3]-buf){
          counter++;
         }
  }
  return counter;
}
void drawLinesAndShapes(Obstacle &obstacle, int short ofColor) {
  // Calculate the positions of lines and shapes based on the obstacle's position
  float lineX = obstacle.x + (obstacle.width / 2);
  float lineY = obstacle.y + 1;
  float length = obstacle.height;
  int numLines = 5;
  float steplength = length /numLines;
  // Draw lines and shapes at the calculated positions
  // Example: Draw a line
  for(int i = 0; i<numLines; i++){
    img.drawLine(lineX, lineY+steplength*i, lineX + 50, lineY+steplength*i, ofColor);
  }
  // Example: Draw a shape (e.g., rectangle)
//  img.fillRect(lineX - 10, lineY - 10, 20, 20, TFT_BLUE);
}

void xmakeObjects(bool remove){
  int short colorOf;
    for (int i = 0; i < maxObjects; i++) {
      if(remove){
        colorOf = backgroundColor;
      }else{
        colorOf = objects[i].color;
      }
    img.fillCircle(objects[i].x, objects[i].y, objects[i].radius, colorOf);
  }
}
void makeObjects(bool remove){
  for (int i = 0; i < maxObjects; i++) {

    if(remove){
      if(objects[i].texture == 0){
        img.fillCircle(objects[i].x, objects[i].y, objects[i].radius, backgroundColor);
      }else{
        img.fillRect(objects[i].x,objects[i].y,objects[i].radius*2-1,objects[i].radius*2-1,backgroundColor);
      }
    }else{
          switch(objects[i].texture){
            case 0:
              img.fillCircle(objects[i].x, objects[i].y, objects[i].radius, objects[i].color);
              break;
            case 1:
              img.pushImage(objects[i].x,objects[i].y,objects[i].radius*2-1,objects[i].radius*2-1,player);
              break;
            case 2:
              img.pushImage(objects[i].x,objects[i].y,objects[i].radius*2-1,objects[i].radius*2-1,enemy);
              break;
          }
        }
  }
  
}

void makeBackGround(){
    img.drawCircle(120, 120, boundaryRadius, TFT_BLUE);
}

void loop() {
  unsigned long startTimeFPS = millis();
  // Check if it's time to change the direction of gravity.
  if (millis() - lastGravityChangeTime >= gravityChangeInterval && gravityEnabled == true) {
    mygravity *= -1;  // Reverse the direction of gravity
    lastGravityChangeTime = millis();  // Update the time of the last gravity change
  }
  // Check if it's time to turn on gravity.
  if (millis() - startTime >= gravityStartTime && gravityEnabled != true) {
    mygravity = gravityG; // Enable gravity
    dampingFactor = dampingFactorD;
    lastGravityChangeTime = millis();
    gravityEnabled = true;
  }

    //makeBackGround();
  
    // Remove the objects.
    makeObjects(true);


  // Remove the obstacles. 
  for (int i = 0; i < maxObstacles; i++) {
    img.drawRoundRect(obstacles[i].x, obstacles[i].y, obstacles[i].width, obstacles[i].height, 4, backgroundColor);
  }
  drawLinesAndShapes(obstacles[0],backgroundColor);

  // Update the position of each obstacle based on velocity.
  for (int i = 0; i < maxObstacles; i++) {
    obstacles[i].x += obstacles[i].vx;
    obstacles[i].y += obstacles[i].vy;
  
  // Check if obstacles hit the left or right boundary.
  if (obstacles[i].x - obstacles[i].width / 2 < 0 || obstacles[i].x + obstacles[i].width / 2 > TFT_WIDTH) {
    obstacles[i].vx = -obstacles[i].vx; // Reverse the velocity to bounce off the boundary.
  }

  // Check if obstacles hit the top or bottom boundary.
  if (obstacles[i].y - obstacles[i].height / 2 < 0 || obstacles[i].y + obstacles[i].height / 2 > TFT_HEIGHT) {
    obstacles[i].vy = -obstacles[i].vy; // Reverse the velocity to bounce off the boundary.
  }
  }
  // Update the motion of each obstacle
  for (int i = 0; i < maxObstacles; i++) {
    updateObstacleMotion(obstacles[i]);
  }
  // Update the position of each object based on velocity and apply damping.
  for (int i = 0; i < maxObjects; i++) {
    objects[i].vx *= dampingFactor;
    objects[i].vy *= dampingFactor;

    objects[i].x += objects[i].vx;
    objects[i].y += objects[i].vy;

    // Apply gravity to each object.
    applyGravity(objects[i]);

    // Apply boundary checks.
//    boundries(objects[i], maxSpriteX, maxSpriteY);
    circularBoundary(objects[i], maxSpriteX, maxSpriteY);
  }

  // Check for collisions and resolve them.
  for (int i = 0; i < maxObjects; i++) {
    for (int j = i + 1; j < maxObjects; j++) {
      float dx = objects[j].x - objects[i].x;
      float dy = objects[j].y - objects[i].y;
      float distance = sqrt(dx * dx + dy * dy);

      if (distance < objects[i].radius + objects[j].radius) {
        // Resolve the collision.
        resolveCollision(objects[i], objects[j]);
      }
    }
  }
  // Check for collisions with the obstacle and resolve them.
  for (int i = 0; i < maxObjects; i++) {

    // Check and resolve collisions with obstacles (similar to object collision code)
    for (int j = 0; j < maxObstacles; j++) {
      if (checkCollisionWithObstacle(objects[i], obstacles[j])) {
        resolveCollisionWithObstacle(objects[i], obstacles[j]);
      }
    }
  }
  // Draw the objects on the screen.
    makeObjects(false);
/*
  for (int i = 0; i < maxObjects; i++) {
    img.fillCircle(objects[i].x, objects[i].y, objects[i].radius, objects[i].color);
  }
*/

  // Draw the obstacle on the screen.
  //img.fillRect(obstacle.x, obstacle.y, obstacle.width, obstacle.height, TFT_RED);
  for (int i = 0; i < maxObstacles; i++) {
    img.drawRoundRect(obstacles[i].x, obstacles[i].y, obstacles[i].width, obstacles[i].height, 4, TFT_RED);
  }
  drawLinesAndShapes(obstacles[0],TFT_WHITE);


  img.pushSprite(5,5);

//  delay(20);  // Adjust the delay to control the simulation speed.
  int CPUclk = getCpuFrequencyMhz();
  unsigned long endTime = millis();
  unsigned long frameTime = endTime - startTimeFPS;
  float fps = 1000.0 / frameTime; // Calculate FPS

  img.fillRect(35, 25, 120, 8, backgroundColor);
  img.fillRect(55, 16, 50, 8, backgroundColor);
//  img.fillRect(35, 25, 120, 8, backgroundColor);
  img.drawString(" Objts: " + String(countBallsinBasket()),55,16,1);
  img.drawString(" FPS: " + String(fps) + " Clk: " + String(CPUclk) + "MHz",35,25,1);

}
