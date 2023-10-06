const movementState = {
  Idle: 1,
  Singular: 2,
  Repeat: 3,
  ReadCSV: 4,
  Realtime: 5
};

const ModeCSV = {
  Singular: 1,
  Repeat: 2
};
let modeCSV = ModeCSV.Singular;
let showKinematicReach = false;

class DeltaRobot extends Robot
{
  constructor(position, rotation, scale, config = {}) {
    super("deltarobot",position,rotation,scale)
    let {upperArmLength: upperArmLength = 0.6, lowerArmLength: lowerArmLength = 1} = config;

        this.name = this.name + this.id; // to avoid having multiple deltarobots with the same name
        
        this.csvPositionCounter = 0;
        this.deltaRobotHash = "b6e472987eb05265db453748cb32ba5d1f178411a2561030aa09e4d60e5102753ffee20e043b0096bbbdabc6d3a38ee1";


        if(lowerArmLength < upperArmLength)
        {
          upperArmLength = Math.sqrt(Math.pow(0.347, 2) + Math.pow(0.0275, 2));
          lowerArmLength = 0.800;
        }
        this.demoMove = false;
        this.demoCurrentAngle = [0,0,0];
        this.demoTargetAngle = [-1,-1,-1];
        this.demoArmSpeed = [0,0,0]; 
        this.movementState = movementState.Idle;
        
        this.baseplateOffsetRadius = 0.200;
        this.upperArmLength = upperArmLength;
        //! this.upperArmLength = Math.sqrt(Math.pow(0.347, 2) + Math.pow(0.0275, 2)); //sometimes also referred to as the proximal links
        this.upperArmMass = 1.04330120;// + 0.3; //upper arm + hose holder for vacuum
        this.upperArmOuterRadius = 0.0600 / 2;
        this.upperArmInnerRadius = 0.0565 / 2;
        this.upperArmElasticModulus = 67 * Math.pow(10, 9);
        this.upperArmInertiaMatrix = new math.matrix([[0.00120470,-0.001247789,0.00056848],[-0.00124778,0.02145074,-0.00002591],[-0.00002591,0.00000022,0.02175818]]); //DPR01, coordinate system acc. to Brinker

        /* Upper arm mechanical properties evaluated from structural testing */
        this.upperArmTorsionalStiffness = 4010;
        this.upperArmHorizontalBendStiffness = 352500;
        this.upperArmVerticalBendStiffness = 410500;
        this.upperArmExtensionalStiffness  = 141780000;

        this.lowerArmOffset = 0.100; //distance between the lower arms

        this.lowerArmLength = lowerArmLength; //sometimes also referred to as the distal links
        //! this.lowerArmLength = lowerArmLength; //sometimes also referred to as the distal links
        this.lowerArmMass = 0.186;// + 0.120; //lower arm + connectionstrip for vacuum
        this.lowerArmOuterRadius = 0.014 / 2;
        this.lowerArmInnerRadius = 0.011 / 2;
        this.lowerArmElasticModulus = 21 * Math.pow(10, 9);
        this.lowerArmShearModulus = Math.pow(10, 10); //asumed extremely high

        this.lowerArmExtensionalStiffness = 73630000;

        this.lowerArmCollisionShape = [new Line(glm.vec3(-.00732,0.02732,.1475),glm.vec3(-.00386,0.02532,.1475)),
                                    new Line(glm.vec3(-.00386,0.02532,.1475),glm.vec3(0.00814,0.04611,.1475)),
                                    new Line(glm.vec3(0.00814,0.04611,.1475),glm.vec3(0.03586,0.03011,.1475)),
                                    new Line(glm.vec3(0.03586,0.03011,.1475),glm.vec3(0.02386,0.00932,.1475)),
                                    new Line(glm.vec3(0.02386,0.00932,.1475),glm.vec3(0.02732,0.00732,.1475)),
                                    new Line(glm.vec3(0.02732,0.00732,.1475),glm.vec3(0.00732,-.02732,.1475)),
                                    new Line(glm.vec3(0.00732,-.02732,.1475),glm.vec3(-.02732,-.00732,.1475)),
                                    new Line(glm.vec3(-.02732,-.00732,.1475),glm.vec3(-.00732,0.02732,.1475))];

        this.balljointAngleMax = glm.radians(43.42); //limit by aluminium cupholder v.s. upperarm, at which the ball joints are mechanically pushed out of place
    
        this.effectorOffsetRadius = 0.045;
        this.minArmAngle = glm.radians(-50) + Math.atan(27.5/347.0);
        this.maxArmAngle = glm.radians(100) + Math.atan(27.5/347.0); //-50 - 100 plus upperarm offset  
        

        //Generic robot stuff
        this.actuatorAngle = new glm.vec3(0, 0, 0); //servo motor position, horizontal upper arm = 0rad
        this.effectorPosition = new glm.vec3(0, 0, 0);

        //used for dynamics
        this.effectorAcceleration = new glm.vec3(0, 0, 0);
        this.effectorMassCenterPoint = new glm.vec3(0, 0, -.416);
        //this.effectorMass = 4.1; //Mass incl. platform and optional product

        //this.effectorMass = 2.912;
        this.effectorMass = 3.262;
        //this.effectorMass = 2.562;

        this.effectorModel = new p5.Geometry(); //Model of the end effector
        this.effectorIzz = 0; //Izz (4th axis coupled mass) and optional product

        this.gearboxTiltStiffness = 773500;//Math.pow(10,17);
        this.gearboxTorsionStiffness = 147800; //Math.pow(10,17);
        this.gearboxInertia = 0.000021 + 0.000123 ; //gearbox inertia as seen from the motor shaft (in kgm^2)
        this.gearboxRatio = 38.5; //gearbox reduction ratio

        this.actuatorEfficiency = 0.94;


        this.actuator = [new ActuatorToGraph, new ActuatorToGraph, new ActuatorToGraph];

        this.effectorPositionOffset = new Array(3);

        this.upperArm = new Array(3);
        this.lowerArm = new Array(6);

        //Appendages
        this.enableAppendages = false;
        this.lowerVacuumduct2 = new Array(3); //vacuum duct connected to the second arm of the delta
        this.lowerVacuumduct3 = new Array(3); //vacuum duct connected to the third arm of the delta
        this.platformVacuumduct2 = new Array(1); //vacuum duct connected to the platform following the second set of lower arms
        this.platformVacuumduct3 = new Array(1); //vacuum duct connected to the platform following the third set of lower arms

        this.encoderCableMount = new Array(10); //cable mounts, starting counting from the servo to the drive (with the platform starting at 0)
        this.encoderCable = new Spline(0);
        this.encoderCableLength = 0.335;
        this.encoderCableSections = 34;

        this.enableMotionplan = false;

        this.realtimeMotionplan = true;
        this.stepMotionplan = 0;

        this.kinematicReach = 0;
        this.mechanicalLimitations = new Array();

        //Color of deltaRobot  
        this.red = Math.floor(Math.random() * 256);
        this.green = Math.floor(Math.random() * 256);
        this.blue = Math.floor(Math.random() * 256);

        //calculate the offset positions for the effectorPosition
        var i;
        for (i = 0; i < 3; i++) {
            this.effectorPositionOffset[i] = glm.vec3(this.effectorOffsetRadius * Math.cos(i * TWO_THIRD_PI), this.effectorOffsetRadius * Math.sin(i * TWO_THIRD_PI), 0.0);
        };

        //calculate the upper arms
        var i;
        for (i = 0; i < 3; i++) {
            this.upperArm[i] = new Line();

            //calculate the position of the servos
            this.upperArm[i].start = glm.vec3(this.baseplateOffsetRadius * Math.cos(i * TWO_THIRD_PI), this.baseplateOffsetRadius * Math.sin(i * TWO_THIRD_PI), 0.0);

            //calculate the position of the elbows
            this.upperArm[i].end = glm.vec3(this.upperArmLength * Math.cos(this.actuatorAngle[i]), 0.0, this.upperArmLength *  Math.sin(this.actuatorAngle[i]));

            this.upperArm[i].end = glm.add((glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](this.upperArm[i].end), this.upperArm[i].start);
        };

        //use the triliterate function to calculate the end effectorPosition position
        this.effectorPosition = triliterate(new Sphere(glm.sub(this.upperArm[0].end, this.effectorPositionOffset[0]), this.lowerArmLength), new Sphere(glm.sub(this.upperArm[1].end, this.effectorPositionOffset[1]), this.lowerArmLength), new Sphere(glm.sub(this.upperArm[2].end, this.effectorPositionOffset[2]), this.lowerArmLength)).end;

        //calculate the positions of the lower arms
        var i;
        for (i = 0; i < 3; i++) {
            this.lowerArm[i*2] = new Line();
            this.lowerArm[(i*2)+1] = new Line();

            this.lowerArm[i*2].start =  glm.sub(this.upperArm[i].end, (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](glm.mul(yAxis(), this.lowerArmOffset/2), i * TWO_THIRD_PI));
            this.lowerArm[(i*2)+1].start =  glm.add(this.upperArm[i].end, (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](glm.mul(yAxis(), this.lowerArmOffset/2), i * TWO_THIRD_PI));

            this.lowerArm[i*2].end =  glm.sub(glm.add(this.effectorPosition, this.effectorPositionOffset[i]), (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](glm.mul(yAxis(), this.lowerArmOffset/2), i * TWO_THIRD_PI));
            this.lowerArm[(i*2)+1].end =  glm.add(glm.add(this.effectorPosition, this.effectorPositionOffset[i]), (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](glm.mul(yAxis(), this.lowerArmOffset/2), i * TWO_THIRD_PI));
        };
        this.calculateAppendages();
        this.calculateKinematicReach(15, 10); // normaal 15 en 10

        //clickable objects
        this.servoID = new Array(3);
        for (i = 0; i < 3; i++) {
            this.servoID[i] = display.clickableObjects + i + 1;

        }
        var obj = this; //fixing the scope
        display.clickableObjectCallback.push(function(){obj.actuator[0].displayData()});
        display.clickableObjectCallback.push(function(){obj.actuator[1].displayData()});
        display.clickableObjectCallback.push(function(){obj.actuator[2].displayData()});

        this.lowerArmID = new Array(6);
        for (i = 0; i < 6; i++) {
            this.lowerArmID[i] = display.clickableObjects + i + 4;
            display.clickableObjectCallback.push(onClick);
        }
        display.clickableObjects += 9;

        function onClick(){
            var hudID = document.getElementById("hud");
            hudID.innerHTML = "Object " + objectAtMouse() + " clicked!";
        };


        for (let i = 0; i < 3; i++)
        {
          let newServo = new Servo({x:this.upperArm[i].start[0],y:this.upperArm[i].start[1],z:this.upperArm[i].start[2]},{z:(i * TWO_THIRD_PI)},{}); //
          this.actuatorList[i] = newServo;
          newServo.Connect(this);
          display.addObject(newServo);
        }
    }
    
    // moves actuator a bit toward the target angle every time it is called
    moveActuator(actuator)  
    {
      const error = (this.demoTargetAngle[actuator] - this.demoCurrentAngle[actuator]);
      this.demoArmSpeed[actuator] = (error * 2) * DeltaTime();  // P controler        // multiply the speed by 1/framerate to make sure the speed stays the same no matter what framerate is used
      this.demoCurrentAngle[actuator] = this.demoCurrentAngle[actuator] + this.demoArmSpeed[actuator];
      this.actuatorList[actuator].SetValue(this.demoCurrentAngle[actuator]);

      return (error < 0.02 && error > -0.02)  // if target reached
    }

    update() {  // -----------------------------------------------------------------------------------------------------------------------UPDATE
        super.update();
        let i = 0;
        this.actuatorList.forEach(servo => {
          this.setActuatorAngle(i,servo.value);
          i++;
        });

        switch (this.movementState) {
          case movementState.Singular:
              let AllPositionsReached = true; // if this variable is still true at the end, all positions are reached
              if (!this.moveActuator(0)) AllPositionsReached = false;
              if (!this.moveActuator(1)) AllPositionsReached = false;
              if (!this.moveActuator(2)) AllPositionsReached = false;
              
              if (AllPositionsReached) {
                this.movementState = movementState.Idle;
              }
            break;
          case movementState.Repeat:
            if (this.moveActuator(0)) {
              if (this.demoTargetAngle[0] === -1){
                this.demoTargetAngle[0] = 1;
              } else {
                this.demoTargetAngle[0] = -1;
              }
            }
            if (this.moveActuator(1)) {
              if (this.demoTargetAngle[1] === -1){
                this.demoTargetAngle[1] = 1;
              } else {
                this.demoTargetAngle[1] = -1;
              }
            }
            if (this.moveActuator(2)) {
              if (this.demoTargetAngle[2] === -1){
                this.demoTargetAngle[2] = 1;
              } else {
                this.demoTargetAngle[2] = -1;
              }
            }
            break;
          case movementState.ReadCSV: // if controlled by CSV file
            csvToArray((results)=>
            {
              if(results.data[0].hash === this.deltaRobotHash) {
                this.csvPositionCounter = 0;
                this.nextPosition(this);
                this.movementState = movementState.Idle;
              }
              else {
                console.log("deze file komt niet overeen met een deltarobot");
              }
            });
          break;
          default:
            break;
        }
    }

    // 'originalobject' is required because the 'this' keyword will point to a different object if this function is called from a different class
    nextPosition(originalObject) {
      csvToArray((results)=>{
        if(originalObject.csvPositionCounter >= results.data.length -1)
        {
          if(modeCSV === ModeCSV.Singular) {
            originalObject.movementState = movementState.Idle;
            return;
          } else {
            originalObject.csvPositionCounter = 0;
          }
        }

        if(originalObject.csvPositionCounter == 5)
        {
          MoveProduct();
        }

        originalObject.actuatorList[0].SetValue(results.data[originalObject.csvPositionCounter].pos0, results.data[originalObject.csvPositionCounter].interval, originalObject.nextPosition,originalObject);
        originalObject.actuatorList[1].SetValue(results.data[originalObject.csvPositionCounter].pos1, results.data[originalObject.csvPositionCounter].interval);
        originalObject.actuatorList[2].SetValue(results.data[originalObject.csvPositionCounter].pos2, results.data[originalObject.csvPositionCounter].interval);
        originalObject.csvPositionCounter++;
      });
    }

    create() {  // -----------------------------------------------------------------------------------------------------------------------CREATE
        super.create();
        mPush();
            noStroke();

            //base cross
            ambientMaterial(0,0,255);
            //!color of object
            specularMaterial(this.red,this.green,this.blue);
            for (var i = 0; i < 3; i++) {
                mPush();
                mTranslate(this.upperArm[i].start[0], this.upperArm[i].start[1], this.upperArm[i].start[2]);
                mRotateZ(i * TWO_THIRD_PI);
                mPush();
                mTranslate(0,0.060,0.015);
                cylinder(0.085, 0.040);
                mPop();
                mTranslate(-0.150,0.060,0.050);
                box(0.300,0.040,0.100);
            mPop();
                mPush();
                mRotateX(HALF_PI);
                mTranslate(0,0.050,0);
                cylinder(0.080, 0.100);
            mPop();
            }

            //servo-reductor
            //noStroke();
            //for (var i = 0; i < 3; i++) {     // servo's are now created by Actuator classes (changed by julian)   -----------------------------------------------------------------
                
                /*
                mPush();
                mTranslate(this.upperArm[i].start[0], this.upperArm[i].start[1], this.upperArm[i].start[2]);
                mRotateZ(i * TWO_THIRD_PI);

                //reductor
                ambientMaterial(127);
                specularMaterial(255);
                mTranslate(0,.0835,0);
                mCylinder(this.servoID[i],0.059, 0.007);
                mTranslate(0,0.0215,0);
                mCylinder(this.servoID[i],0.0475, 0.036);
                mTranslate(0,0.0285,0);
                mCone(this.servoID[i],0.0475,0.021);
                mTranslate(0,-0.001,0);
                mCylinder(this.servoID[i],0.034,0.019);
                mTranslate(0,0.0225,0);
                mBox(this.servoID[i],0.082,0.026,0.082);

                //servo
                ambientMaterial(32);
                specularMaterial(32);
                mTranslate(0,0.0165,0);
                mBox(this.servoID[i],0.082,0.007,0.082);
                mTranslate(0,0.0705,0);
                mBox(this.servoID[i],0.082,0.134,0.054);
                mBox(this.servoID[i],0.054,0.134,0.082);
                mTranslate(0,0.083,0);
                mCylinder(this.servoID[i],0.035,0.032);
                mTranslate(0,0.02125,0);
                mCylinder(this.servoID[i],0.030,0.0105);
                mPop();*/
            //}

            //upper arms
            noStroke();
            ambientMaterial(32);
            specularMaterial(32);

            for (var i = 0; i < 3; i++) {
                mPush();
                mTranslate(this.upperArm[i].start[0], this.upperArm[i].start[1], this.upperArm[i].start[2]);
                mRotateZ(i * TWO_THIRD_PI + HALF_PI);
                mRotateX(-this.actuatorAngle[i] - Math.atan(27.5/347.0));
                mPush();
                mTranslate(0,-this.upperArm[i].length()/2,0);
                cylinder(0.030, this.upperArm[i].length());
                mPop();
                mTranslate(0,-0.0315,0)
                cylinder(0.035, 0.063);
                mPop();

                mPush();
                mTranslate(this.upperArm[i].start[0], this.upperArm[i].start[1], this.upperArm[i].start[2]);
                mRotateZ(i * TWO_THIRD_PI);
                mTranslate(0,0.010,0);
                cylinder(0.035, 0.090);
                mPop();
            }

            //lower arms
            for (var i = 0; i < 6; i++) {
                ambientMaterial(127);
                specularMaterial(255);
                mPush();
                mTranslate(this.lowerArm[i].start[0], this.lowerArm[i].start[1], this.lowerArm[i].start[2]);
                mSphere(this.lowerArmID[i],0.0125);
                mPop();
                mPush();
                mTranslate(this.lowerArm[i].end[0], this.lowerArm[i].end[1], this.lowerArm[i].end[2]);
                mSphere(this.lowerArmID[i],0.0125);
                mPop();

                ambientMaterial(32);
                specularMaterial(32);
                mPush();
                mTranslate((this.lowerArm[i].start[0] + this.lowerArm[i].end[0])/2, (this.lowerArm[i].start[1] + this.lowerArm[i].end[1])/2, (this.lowerArm[i].start[2] + this.lowerArm[i].end[2])/2);
                mRotate(-this.lowerArm[i].angle(yAxis()), createVector(glm.cross(this.lowerArm[i].unit(),yAxis())[0], glm.cross(this.lowerArm[i].unit(),yAxis())[1], glm.cross(this.lowerArm[i].unit(),yAxis())[2]));
                mCylinder(this.lowerArmID[i], this.lowerArmOuterRadius, this.lowerArm[i].length());
                mPop();
            }

            //platform
            specularMaterial(32);
            for (var i = 0; i < 3; i++) {
                mPush();
                mTranslate(this.effectorPosition[0], this.effectorPosition[1], this.effectorPosition[2]);
                mRotateZ(i * TWO_THIRD_PI);
                mTranslate(0.005,0,0);
                box(0.094,0.064,0.014);
                mPop();
            }
            mPush();
            mTranslate(this.effectorPosition[0], this.effectorPosition[1], this.effectorPosition[2]);
            mRotateX(HALF_PI);
            mTranslate(0,-0.0355,0);
            cylinder(0.029,0.057);
            specularMaterial(255);
            mTranslate(0,-0.0385,0);
            cylinder(0.020,0.014);
            mPop();
            
            //4th axis servo
            mPush();
            mTranslate(this.effectorPosition[0], this.effectorPosition[1], this.effectorPosition[2]);
            mPush();
            mRotateZ(ONE_THIRD_PI);
            
            ambientMaterial(127);
            specularMaterial(255);
            mTranslate(0,0,0.007 + 0.0025);
            box(0.04,0.04,0.005);
            
            ambientMaterial(32);
            specularMaterial(32);
            mTranslate(0,0,0.0025 + 0.01725);
            mRotateX(HALF_PI);
            cylinder(0.0195,0.0345);
            mRotateX(-HALF_PI);
            
            mTranslate(0,0,0.01725 + 0.0045);
            box(0.04,0.04,0.009);
            
            ambientMaterial(127);
            specularMaterial(255);
            mTranslate(0,0,0.034 + 0.0045);
            box(0.04,0.04,0.068);
            
            ambientMaterial(32);
            specularMaterial(32);
            mTranslate(0,0,0.034 + 0.012);
            box(0.04,0.04,0.024);

            mTranslate(0.032,0,0);
            box(0.024,0.032,0.024);
            mPop();
            
            //body on the platform used for collision checking
            
            noFill();
            strokeWeight(1);
            stroke(color(255, 0, 0, 255));
            for(var i = 0; i < this.lowerArmCollisionShape.length; i++){
                this.lowerArmCollisionShape[i].display();
            };
            
            mPop();
            
            if(this.enableAppendages == true){
                stroke(color(0, 192, 192, 255));
                var i;
                for (i = 0; i < 3; i++) {
                this.lowerVacuumduct2[i].display();
                this.lowerVacuumduct3[i].display();
                }
                line(this.platformVacuumduct2[0].start[0], this.platformVacuumduct2[0].start[1],this.platformVacuumduct2[0].start[2],this.platformVacuumduct2[0].end[0],this.platformVacuumduct2[0].end[1],this.platformVacuumduct2[0].end[2]);
                this.platformVacuumduct2[0].display();
                this.platformVacuumduct3[0].display();

                stroke(color(255,0,0,255));
                this.encoderCable.optimizeKallayMEC();
                this.encoderCable.display();
            };
            
            //show motionplan with spheres and line
            if(this.enableMotionplan == true){
                if (this.motionPlan.motionStep.length > 0){
                mPush();
                ambientMaterial(0,255,0);
                mTranslate(this.motionPlan.motionStep[0].targetPosition[0],this.motionPlan.motionStep[0].targetPosition[1],this.motionPlan.motionStep[0].targetPosition[2]);
                sphere(0.020);
                mPop();
                mPush();
                ambientMaterial(255,0,0);
                mTranslate(this.motionPlan.motionStep[this.motionPlan.motionStep.length - 1].targetPosition[0],this.motionPlan.motionStep[this.motionPlan.motionStep.length - 1].targetPosition[1],this.motionPlan.motionStep[this.motionPlan.motionStep.length -1].targetPosition[2]);
                sphere(0.020);
                mPop();
                }
                for(var n = 1; n < this.motionPlan.motionStep.length-1; n++){
                mPush();

                ambientMaterial(255,255,0);

                mTranslate(this.motionPlan.motionStep[n].targetPosition[0],this.motionPlan.motionStep[n].targetPosition[1],this.motionPlan.motionStep[n].targetPosition[2]);
                sphere(0.020);
                mPop();
                };

                //display motion path of the delta
                strokeWeight(.001);
                stroke(color(0, 0, 255, 255));
                this.motionPlan.motionPath.display();

                //update position
                if (this.realtimeMotionplan == true){
                t = (((new Date()).getMilliseconds() / 10) + ((new Date()).getSeconds() * 100)) % this.motionPlan.motionPath.point.length;
                }else{
                this.stepMotionplan = (this.stepMotionplan + 1) % this.motionPlan.motionPath.point.length;
                t = this.stepMotionplan;
                }
                this.setEffectorPosition(this.motionPlan.motionPath.point[Math.round(t)]);
            }

            
            // ! stroke(51);
            if(showKinematicReach) {
            ambientMaterial(127,127,255,63);
            specularMaterial(127,127,255,63);
            model(this.kinematicReach);
            }
            // console.log(this, "reach:",this.kinematicReach);

            // ambientMaterial(127,0,0,127);
            // specularMaterial(127,0,0,127);
            // for(var i = 0; i < this.mechanicalLimitations.length; i++){
            //     model(this.mechanicalLimitations[i]);
            // };
            
            //noStroke();
            mPop();
    }

    setColor(red, green, blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }
    setEnableAppendages() {
        this.enableAppendages = true;
        this.calculateAppendages();
        this.encoderCable = mecKallay(glm.add(this.encoderCableMount[1], zAxis()), this.platformVacuumduct2[0].unit(), this.encoderCableMount[2], this.lowerVacuumduct2[0].unit(), this.encoderCableLength, this.encoderCableSections, 512);
      }
      resetEnableAppendages() {
        this.enableAppendages = false;
      }
      calculateAppendages() {
        //ducts connected to the platform
        this.platformVacuumduct2[0] = new Line(glm.sub(this.effectorPosition, glm.mul(zAxis(), 25)), glm.sub(glm.add(this.effectorPosition, this.effectorPositionOffset[1]), glm.mul(zAxis(), 25)));
        this.platformVacuumduct2[0].end = glm.add(this.platformVacuumduct2[0].start, glm.mul(this.platformVacuumduct2[0].unit(), 77));
    
        this.platformVacuumduct3[0] = new Line(glm.sub(this.effectorPosition, glm.mul(zAxis(), 25)), glm.sub(glm.add(this.effectorPosition, this.effectorPositionOffset[2]), glm.mul(zAxis(), 25)));
        this.platformVacuumduct3[0].end = glm.add(this.platformVacuumduct3[0].start, glm.mul(this.platformVacuumduct3[0].unit(), 77));
    
        //calculate the ductwork on the 2nd arm
        this.lowerVacuumduct2[1] = new Line(glm.add(this.effectorPosition, this.effectorPositionOffset[1]), this.upperArm[1].end); //center piece runs parallel to the lower arms
        this.lowerVacuumduct2[1].start = glm.add(this.lowerVacuumduct2[1].start, glm.mul(this.lowerVacuumduct2[1].unit(), 274.5)); //lower hinge point
        this.lowerVacuumduct2[1].end = glm.sub(this.lowerVacuumduct2[1].end, glm.mul(this.lowerVacuumduct2[1].unit(), 215.5)); //upper hinge point
    
        this.lowerVacuumduct2[1].start = glm.add(this.lowerVacuumduct2[1].start, glm.mul(glm.cross((new Line(this.lowerArm[2].position(0.5), this.lowerArm[3].position(0.5))).unit(), this.lowerVacuumduct2[1].unit()), 44)); //distance between arms and center of the vacuum duct
        this.lowerVacuumduct2[1].end = glm.add(this.lowerVacuumduct2[1].end, glm.mul(glm.cross((new Line(this.lowerArm[2].position(0.5), this.lowerArm[3].position(0.5))).unit(), this.lowerVacuumduct2[1].unit()), 44)); //distance between arms and center of the vacuum duct
    
        this.lowerVacuumduct2[1].start = glm.sub(this.lowerVacuumduct2[1].start, glm.mul(this.lowerVacuumduct2[1].unit(), 21.04143057)); //connection offset near platform
        this.lowerVacuumduct2[1].end = glm.add(this.lowerVacuumduct2[1].end, glm.mul(this.lowerVacuumduct2[1].unit(), 20.96776981)); //connection offset near upper arm
    
        this.lowerVacuumduct2[0] = new Line(this.lowerVacuumduct2[1].end, this.lowerVacuumduct2[1].start); //initialize with direction inline with next part of the duct
        this.lowerVacuumduct2[0].start = glm.sub(this.lowerVacuumduct2[0].end, glm.mul((glm.angleAxis(glm.radians(38.0), glm.cross(glm.cross((new Line(this.lowerArm[2].position(0.5), this.lowerArm[3].position(0.5))).unit(), this.lowerVacuumduct2[1].unit()), this.lowerVacuumduct2[1].unit())))['*'](this.lowerVacuumduct2[1].unit()), 70.58682409)); //connection piece
        this.lowerVacuumduct2[0].end = this.lowerVacuumduct2[1].start; //connect the end of the connection piece to the start of the center piece
    
        this.lowerVacuumduct2[2] = new Line(this.lowerVacuumduct2[1].end, this.lowerVacuumduct2[1].start); //initialize with direction inline with next part of the duct
        this.lowerVacuumduct2[2].end = glm.sub(this.lowerVacuumduct2[2].start, glm.mul((glm.angleAxis(glm.radians(130.0), glm.cross((new Line(this.lowerArm[2].position(0.5), this.lowerArm[3].position(0.5))).unit(), this.lowerVacuumduct2[1].unit())))['*'](this.lowerVacuumduct2[1].unit()), 71)); //connection piece
    
    
        //calculate the ductwork on the 3rd arm
        this.lowerVacuumduct3[1] = new Line(glm.add(this.effectorPosition, this.effectorPositionOffset[2]), this.upperArm[2].end); //center piece runs parallel to the lower arms
        this.lowerVacuumduct3[1].start = glm.add(this.lowerVacuumduct3[1].start, glm.mul(this.lowerVacuumduct3[1].unit(), 274.5)); //lower hinge point
        this.lowerVacuumduct3[1].end = glm.sub(this.lowerVacuumduct3[1].end, glm.mul(this.lowerVacuumduct3[1].unit(), 215.5)); //upper hinge point
    
        this.lowerVacuumduct3[1].start = glm.add(this.lowerVacuumduct3[1].start, glm.mul(glm.cross((new Line(this.lowerArm[4].position(0.5), this.lowerArm[5].position(0.5))).unit(), this.lowerVacuumduct3[1].unit()), 44)); //distance between arms and center of the vacuum duct
        this.lowerVacuumduct3[1].end = glm.add(this.lowerVacuumduct3[1].end, glm.mul(glm.cross((new Line(this.lowerArm[4].position(0.5), this.lowerArm[5].position(0.5))).unit(), this.lowerVacuumduct3[1].unit()), 44)); //distance between arms and center of the vacuum duct
    
        this.lowerVacuumduct3[1].start = glm.sub(this.lowerVacuumduct3[1].start, glm.mul(this.lowerVacuumduct3[1].unit(), 21.04143057)); //connection offset near platform
        this.lowerVacuumduct3[1].end = glm.add(this.lowerVacuumduct3[1].end, glm.mul(this.lowerVacuumduct3[1].unit(), 23.53492059)); //connection offset near upper arm
    
        this.lowerVacuumduct3[0] = new Line(this.lowerVacuumduct3[1].end, this.lowerVacuumduct3[1].start); //initialize with direction inline with next part of the duct
        this.lowerVacuumduct3[0].start = glm.sub(this.lowerVacuumduct3[0].end, glm.mul((glm.angleAxis(glm.radians(38.0), glm.cross(glm.cross((new Line(this.lowerArm[4].position(0.5), this.lowerArm[5].position(0.5))).unit(), this.lowerVacuumduct3[1].unit()), this.lowerVacuumduct3[1].unit())))['*'](this.lowerVacuumduct3[1].unit()), 70.58682409)); //connection piece
        this.lowerVacuumduct3[0].end = this.lowerVacuumduct3[1].start; //connect the end of the connection piece to the start of the center piece
    
        this.lowerVacuumduct3[2] = new Line(this.lowerVacuumduct3[1].end, this.lowerVacuumduct3[1].start); //initialize with direction inline with next part of the duct
        this.lowerVacuumduct3[2].end = glm.sub(this.lowerVacuumduct3[2].start, glm.mul((glm.angleAxis(glm.radians(-145.0), glm.cross((new Line(this.lowerArm[4].position(0.5), this.lowerArm[5].position(0.5))).unit(), this.lowerVacuumduct3[1].unit())))['*'](this.lowerVacuumduct3[1].unit()), 71)); //connection piece
    
        this.encoderCableMount[1] = glm.add(glm.add(this.platformVacuumduct2[0].end, glm.mul(zAxis(), -3.5)), glm.sub(glm.mul(this.platformVacuumduct2[0].unit(), -5.0), glm.mul((new Line(this.lowerArm[2].end, this.lowerArm[3].end)).unit(), -29.5)));
        this.encoderCableMount[2] = glm.sub(glm.add(glm.add(this.lowerVacuumduct2[0].start, glm.mul(glm.cross(this.lowerVacuumduct2[1].unit(), this.lowerVacuumduct2[0].unit()), -25.5)), glm.mul(glm.cross(glm.cross(this.lowerVacuumduct2[1].unit(), this.lowerVacuumduct2[0].unit()), this.lowerVacuumduct2[1].unit()), -3.5)), glm.mul(this.lowerVacuumduct2[0].unit(), -38.06));
      }
      
      setActuatorAngle(id, angle) {
        this.actuatorList[id].value = angle; // to link the positions to the robot superclass' position storage

        /**
        * Sets an actuator to the desired position and calculates the kinematics by the triliteration of 3 spheres
        *
        * @param id 0..2
        * @param angle in radians
        */
        //set the specified actuator to the desired angle
        this.actuatorAngle[id] = angle;
        
        //re-calculate the position of the elbow
        this.upperArm[id].end[0] = this.upperArmLength * Math.cos(this.actuatorAngle[id]);
        this.upperArm[id].end[1] = 0.0;
        this.upperArm[id].end[2] = this.upperArmLength * Math.sin(this.actuatorAngle[id]);
        this.upperArm[id].end = glm.add((glm.angleAxis(id * TWO_THIRD_PI, zAxis()))['*'](this.upperArm[id].end), this.upperArm[id].start);
    
        //use the triliterate function to calculate the end effectorPosition position
        this.effectorPosition = triliterate(new Sphere(glm.sub(this.upperArm[0].end, this.effectorPositionOffset[0]), this.lowerArmLength), new Sphere(glm.sub(this.upperArm[1].end, this.effectorPositionOffset[1]), this.lowerArmLength), new Sphere(glm.sub(this.upperArm[2].end, this.effectorPositionOffset[2]), this.lowerArmLength)).end;
    
        //calculate the positions of the lower arms
        for (var i = 0; i < 3; i++) {
          this.lowerArm[i * 2] = new Line();
          this.lowerArm[(i * 2) + 1] = new Line();
    
          this.lowerArm[i * 2].start = glm.sub(this.upperArm[i].end, (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](glm.mul(yAxis(), this.lowerArmOffset / 2), i * TWO_THIRD_PI));
          this.lowerArm[(i * 2) + 1].start = glm.add(this.upperArm[i].end, (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](glm.mul(yAxis(), this.lowerArmOffset / 2), i * TWO_THIRD_PI));
    
          this.lowerArm[i * 2].end = glm.sub(glm.add(this.effectorPosition, this.effectorPositionOffset[i]), (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](glm.mul(yAxis(), this.lowerArmOffset / 2), i * TWO_THIRD_PI));
          this.lowerArm[(i * 2) + 1].end = glm.add(glm.add(this.effectorPosition, this.effectorPositionOffset[i]), (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](glm.mul(yAxis(), this.lowerArmOffset / 2), i * TWO_THIRD_PI));
        };
        if (this.enableAppendages == true) {
          this.calculateAppendages();
          this.encoderCable = mecKallay(glm.add(this.encoderCableMount[1], zAxis()), this.platformVacuumduct2[0].unit(), this.encoderCableMount[2], this.lowerVacuumduct2[0].unit(), this.encoderCableLength, this.encoderCableSections, 512);
        }
      }
      setEffectorPosition(effectorPosition) {
        /**
        * Sets the end effectorPosition to the desired position and calculates the inverse kinematics by circle-sphere intersections
        */
        if (effectorPosition instanceof glm.vec3) {
          if (this.inKinematicWorkArea(effectorPosition)) {
            this.effectorPosition = effectorPosition;
          }
        };
    
        //use circle sphere intersection to calculate the position of the upper arms
        for (var i = 0; i < 3; i++) {
          this.upperArm[i].end = (new Sphere(glm.add(this.effectorPosition, this.effectorPositionOffset[i]), this.lowerArmLength)).intersect(new Circle(this.upperArm[i].start, (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](yAxis()), this.upperArmLength)).start;
          this.actuatorAngle[i] = (this.upperArm[i].unit()[2] / Math.abs(this.upperArm[i].unit()[2])) * Math.acos(glm.dot(this.upperArm[i].unit(), (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](xAxis())));
        }
    
        //calculate the positions of the lower arms
        for (var i = 0; i < 3; i++) {
          this.lowerArm[i * 2] = new Line();
          this.lowerArm[(i * 2) + 1] = new Line();
    
          this.lowerArm[i * 2].start = glm.sub(this.upperArm[i].end, (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](glm.mul(yAxis(), this.lowerArmOffset / 2), i * TWO_THIRD_PI));
          this.lowerArm[(i * 2) + 1].start = glm.add(this.upperArm[i].end, (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](glm.mul(yAxis(), this.lowerArmOffset / 2), i * TWO_THIRD_PI));
    
          this.lowerArm[i * 2].end = glm.sub(glm.add(this.effectorPosition, this.effectorPositionOffset[i]), (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](glm.mul(yAxis(), this.lowerArmOffset / 2), i * TWO_THIRD_PI));
          this.lowerArm[(i * 2) + 1].end = glm.add(glm.add(this.effectorPosition, this.effectorPositionOffset[i]), (glm.angleAxis(i * TWO_THIRD_PI, zAxis()))['*'](glm.mul(yAxis(), this.lowerArmOffset / 2), i * TWO_THIRD_PI));
        };
    
        if (this.enableAppendages == true) {
          this.calculateAppendages();
          this.encoderCable = mecKallay(glm.add(this.encoderCableMount[1], zAxis()), this.platformVacuumduct2[0].unit(), this.encoderCableMount[2], this.lowerVacuumduct2[0].unit(), this.encoderCableLength, this.encoderCableSections, 512);
        }
      }
      calculateKinematicReach(detailX, detailY) {
        this.kinematicReach = new p5.Geometry(detailX, detailY);
        var topConeZ = -((Math.sin(this.minArmAngle) * this.upperArmLength) + this.lowerArmLength); //top cut off (downwards from mount)
        var xOffset = (this.baseplateOffsetRadius - this.effectorOffsetRadius) + (this.upperArmLength * Math.cos(this.maxArmAngle)); //Sphere shape center
        var zOffset = -this.upperArmLength * Math.sin(Math.PI - this.maxArmAngle); //Sphere shape center
        var bottomConeZ = -(Math.sqrt(Math.pow(this.lowerArmLength, 2) - Math.pow(xOffset, 2))); //Lowest point of the cone
    
        for (var i = 0; i <= detailY; i++) {
          var v = (i * (((zOffset + bottomConeZ) - topConeZ) / detailY)) + topConeZ; //v goes from top of the cone to the bottom of the cone
          var circleRadius = (new Sphere(glm.vec3(0, 0, zOffset), this.lowerArmLength)).intersect(new Plane(glm.vec3(0, 0, v), zAxis())).radius; //circle radius on this height level
          var startAngle = -ONE_THIRD_PI + Math.asin(Math.sin(ONE_THIRD_PI) * xOffset / circleRadius);
          var endAngle = ONE_THIRD_PI - Math.asin(Math.sin(ONE_THIRD_PI) * xOffset / circleRadius);
    
          for (var j = 0; j <= detailX; j++) {
            var w = Math.floor(3 * (detailX - j) / detailX); //which part of the circle
            var circleCenter = glm.vec3(-xOffset * Math.cos(w * TWO_THIRD_PI), -xOffset * Math.sin(w * TWO_THIRD_PI), v);
            if (w < 3) {
              var u = (((endAngle - startAngle) / (detailX / 3)) * ((detailX - j) % (detailX / 3))) + startAngle;
            } else {
              var u = startAngle;
            }
    
            var p = glm.mul((glm.angleAxis(TWO_THIRD_PI, zAxis())), glm.add((glm.angleAxis(w * TWO_THIRD_PI, zAxis()))['*'](glm.vec3(circleRadius * Math.cos(u), circleRadius * Math.sin(u), 0)), circleCenter));
            if (i < detailY) {
              this.kinematicReach.vertices.push(new p5.Vector(p[0], p[1], p[2]));
              this.kinematicReach.vertexNormals.push(new p5.Vector(p[0], p[1], p[2]));
              this.kinematicReach.uvs.push(u, v);
            } else {
              this.kinematicReach.vertices.push(new p5.Vector(0, 0, (zOffset + bottomConeZ)));
              this.kinematicReach.vertexNormals.push(new p5.Vector(0, 0, (zOffset + bottomConeZ)));
              this.kinematicReach.uvs.push(u, v);
            }
          }
        }
        this.kinematicReach.computeFaces();
        this.kinematicReach.computeNormals();
        this.kinematicReach._makeTriangleEdges()._edgesToVertices();
        this.kinematicReach.gid = this.id; //to give a unique identifier
        //console.log(this.kinematicReach);
      }
      calculateMechanicalLimitations(detailX, detailY) {
        var that = this; //because of the nested functions using the RobotDelta object parameters...
    
        function lowerArmCollision(x1, y1, z1, x2, y2, z2) {
          let limitationSurfaces = [new p5.Geometry(detailX, detailY), new p5.Geometry(detailX, detailY), new p5.Geometry(detailX, detailY), new p5.Geometry(detailX, detailY), new p5.Geometry(detailX, detailY), new p5.Geometry(detailX, detailY)];
          for (var i = 0; i < 6; i++) {
            let limitationProjection = createArray(detailX + 1, detailY + 1);
            for (var k = 0; k <= detailX; k++) { //detailX is interpeted as divisions on the line piece
    
              var collisionPoint = glm.vec3(x1 + ((k / detailX) * (x2 - x1)), y1 + ((k / detailX) * (y2 - y1)), z1 + ((k / detailX) * (z2 - z1)));
    
              //to add: check if collision point is whithin the length of the lower arm
              var balljointProjection = (new Plane(collisionPoint, glm.normalize(new glm.vec3(x2 - x1, y2 - y1, z2 - z1)))).project(glm.sub(that.lowerArm[i].end, that.effectorPosition)); //project end of the lower arm (balljoint) on the collision plane (collision point with the line piece as normal)
    
              var tangencyPoints = (new Circle(collisionPoint, glm.normalize(new glm.vec3(x2 - x1, y2 - y1, z2 - z1)), that.lowerArmOuterRadius)).tangencyPoints(balljointProjection); //tangencyPoints with circle (based on collision point, lower arm radius, direction of the line under investigation) 
              var directionVector = new glm.vec3();
              if (tangencyPoints.start.z < tangencyPoints.end.z) { //highest found tangency point is on the center line of the arm at the collision point situation
                directionVector = (new Line(glm.sub(that.lowerArm[i].end, that.effectorPosition), tangencyPoints.start)).unit(); //vector from balljoint to tangencypoint
              } else {
                directionVector = (new Line(glm.sub(that.lowerArm[i].end, that.effectorPosition), tangencyPoints.end)).unit();
              }
    
              //directionVector = (new Line(glm.sub(that.lowerArm[i].end,that.effectorPosition), collisionPoint)).unit(); //simple solution without lowerArm thickness
              for (var j = 0; j <= detailY; j++) { //detailY is interpeted as angular division for the upper arms
                var upperArmAngle = that.minArmAngle + ((j / detailY) * (that.maxArmAngle - that.minArmAngle));
    
                //calculate ECP based on armAngle via (direction vector * armLength) + effectorPositionOffset
                var ECP = (glm.angleAxis(floor(i / 2) * TWO_THIRD_PI, zAxis()))['*'](glm.vec3(that.upperArmLength * Math.cos(upperArmAngle), 0.0, that.upperArmLength * Math.sin(upperArmAngle)));
                ECP = glm.add(that.upperArm[floor(i / 2)].start, ECP);
                ECP = glm.sub(ECP, glm.mul(directionVector, that.lowerArmLength));
                ECP = glm.sub(ECP, that.effectorPositionOffset[floor(i / 2)]);
    
                limitationProjection[k][j] = ECP;
              }
            }
            for (var k = 0; k <= detailY; k++) { //add area to limitationSurface
              for (var j = 0; j <= detailX; j++) {
                limitationSurfaces[i].vertices.push(new p5.Vector(limitationProjection[j][k]['x'], limitationProjection[j][k]['y'], limitationProjection[j][k]['z']));
                limitationSurfaces[i].vertexNormals.push(new p5.Vector(limitationProjection[j][k]['x'], limitationProjection[j][k]['y'], limitationProjection[j][k]['z']));
                limitationSurfaces[i].uvs.push(k / detailY, j / detailX);
              }
            }
            limitationSurfaces[i].computeFaces();
            limitationSurfaces[i].computeNormals();
            limitationSurfaces[i]._makeTriangleEdges()._edgesToVertices();
    
          }
          return limitationSurfaces;
        }
    
        function lowerArmAngleLimit(a) {
          //fast approach
          let limitationSurfaces = [new p5.Geometry(1, 1), new p5.Geometry(1, 1), new p5.Geometry(1, 1), new p5.Geometry(1, 1), new p5.Geometry(1, 1), new p5.Geometry(1, 1)];
    
          var topConeZ = -((Math.sin(that.minArmAngle) * that.upperArmLength) + that.lowerArmLength); //top cut off (downwards from mount)
          var xOffset = (that.baseplateOffsetRadius - that.effectorOffsetRadius) + (that.upperArmLength * Math.cos(that.maxArmAngle)); //Sphere shape center
          var zOffset = -that.upperArmLength * Math.sin(Math.PI - that.maxArmAngle); //Sphere shape center
          var bottomConeZ = -(Math.sqrt(Math.pow(that.lowerArmLength, 2) - Math.pow(xOffset, 2))); //Lowest point of the cone
    
          for (var i = 0; i < 6; i++) {
            for (let k = 0; k <= 1; k++) {
              for (let j = 0; j <= 1; j++) {
                p = new p5.Vector(
                  (Math.cos(i * ONE_THIRD_PI) * ((j - 0.5) * (that.lowerArmLength * Math.cos(that.balljointAngleMax) * 1.1547))) - (Math.sin(i * ONE_THIRD_PI) * (that.lowerArmLength * Math.cos(that.balljointAngleMax))),
                  (Math.sin(i * ONE_THIRD_PI) * ((j - 0.5) * (that.lowerArmLength * Math.cos(that.balljointAngleMax) * 1.1547))) + (Math.cos(i * ONE_THIRD_PI) * (that.lowerArmLength * Math.cos(that.balljointAngleMax))),
                  topConeZ + (k * (bottomConeZ + zOffset - topConeZ))
                );
                console.log(that.lowerArmLength * Math.cos(that.balljointAngleMax));
                limitationSurfaces[i].vertices.push(p);
                limitationSurfaces[i].uvs.push(j, k);
              }
            }
            limitationSurfaces[i].computeFaces();
            limitationSurfaces[i].computeNormals();
            limitationSurfaces[i]._makeTriangleEdges()._edgesToVertices();
    
          }
          return limitationSurfaces;
    
        }
    
    
        for (var i = 0; i < this.lowerArmCollisionShape.length; i++) { //lower arm with body on platform
          this.mechanicalLimitations = this.mechanicalLimitations.concat(lowerArmCollision(this.lowerArmCollisionShape[i].start['x'], this.lowerArmCollisionShape[i].start['y'], this.lowerArmCollisionShape[i].start['z'], this.lowerArmCollisionShape[i].end['x'], this.lowerArmCollisionShape[i].end['y'], this.lowerArmCollisionShape[i].end['z']));
        }
    
        this.mechanicalLimitations = this.mechanicalLimitations.concat(lowerArmAngleLimit(this.balljointAngleMax));
    
        for (var i = 0; i < this.mechanicalLimitations.length; i++) {
          this.mechanicalLimitations[i].gid = `limitSurface|${i}`; //give all the surfaces a unique gid, might be needed to extend
        };
      }
      inKinematicWorkArea(point) {
        /**
        * Checks if the point given by Matrix p is within the kinematic reach.
        *
        * @return true when Matrix p is in the kinematic reach area.
        */
        var topConeZ = -((Math.sin(this.minArmAngle) * this.upperArmLength) + this.lowerArmLength); //top cut off (downwards from mount)
        var xOffset = (this.baseplateOffsetRadius - this.effectorOffsetRadius) + (this.upperArmLength * Math.cos(this.maxArmAngle)); //Sphere shape center
        var zOffset = -this.upperArmLength * Math.sin(Math.PI - this.maxArmAngle); //Sphere shape center
    
        if (point[2] > topConeZ) { return false; } //above cone
        if (((new Line(point, glm.vec3(-xOffset * Math.cos(0 * TWO_THIRD_PI), -xOffset * Math.sin(0 * TWO_THIRD_PI), zOffset))).length() <= this.lowerArmLength)
          && ((new Line(point, glm.vec3(-xOffset * Math.cos(1 * TWO_THIRD_PI), -xOffset * Math.sin(1 * TWO_THIRD_PI), zOffset))).length() <= this.lowerArmLength)
          && ((new Line(point, glm.vec3(-xOffset * Math.cos(2 * TWO_THIRD_PI), -xOffset * Math.sin(2 * TWO_THIRD_PI), zOffset))).length() <= this.lowerArmLength)) {
          return true;
        } else {
          return false;
        }
      }
      calculateActuators() {
        /**
        * Computes the actuator angle, angular velocity and angular acceleration based on the given motionplan
        * id is the axis number for which the calculation is performed
        * returns a spline with the points representing: angle (x), angular velocity(y) and angular acceleration(z)
        */
        for (var i = 0; i < this.motionPlan.motionPath.point.length; i++) {
          var p = new math.zeros(3, 3);
          p._data[0][0] = this.motionPlan.motionPath.point[i][0];
          p._data[1][0] = -this.motionPlan.motionPath.point[i][1];
          p._data[2][0] = -this.motionPlan.motionPath.point[i][2];
    
          p._data[0][1] = this.motionPlan.motionVelocity.point[i][0];
          p._data[1][1] = -this.motionPlan.motionVelocity.point[i][1];
          p._data[2][1] = -this.motionPlan.motionVelocity.point[i][2];
    
          p._data[0][2] = this.motionPlan.motionAcceleration.point[i][0];
          p._data[1][2] = -this.motionPlan.motionAcceleration.point[i][1];
          p._data[2][2] = -this.motionPlan.motionAcceleration.point[i][2];
          p = this.getActuatorDynamics(p);
          for (var j = 0; j < 3; j++) {
            this.actuator[j].time.push(i / 167);
            this.actuator[j].angularPosition.push(-p._data[j][0] * this.gearboxRatio);
            this.actuator[j].angularVelocity.push(p._data[j][1] * this.gearboxRatio);
            this.actuator[j].angularAcceleration.push(p._data[j][2] * this.gearboxRatio);
            this.actuator[j].applicationTorque.push(((p._data[j][3] / this.gearboxRatio) + (p._data[j][2] * this.gearboxRatio * this.gearboxInertia)) / this.actuatorEfficiency);;
          }
        }
      }
      getActuatorDynamics(effectorDynamics) {
        /**
        * Computes the actuator dynamics based on the given effector dynamics
        * effectorDynamics consists of a 3 x 3 elements:
        * Px  Vx  Ax
        * Py  Vy  Ay
        * Pz  Vz  Az
        *
        * The function returns a matrix with angular values:
        * theta0  omega0  alpha0  torque0
        * theta1  omega1  alpha1  torque1
        * theta2  omega2  alpha2  torque2
        *
        * Procedure according to: J.Brinker, B.Corves and M.Wahle; A Comparative Study of Inverse Dynamics based on Clavel's Delta robot
        */
        var actuatorDynamics = new math.zeros(3, 4);
        var ir = new math.zeros(3, 3); //leg specific auxiliary vectors
        var phi = [new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //angles and auxiliary angles
        var sA = [new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //actuation axis
        var l_1 = [new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //upper arm vectors
        var l_2 = [new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //lower arm vectors
        var omega_1 = [new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //angular velocity of the proximal link
        var omega_2 = [new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //angular velocity of the distal link
        var alpha_1 = [new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //angular acceleration of the proximal link
        var alpha_2 = [new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //angular acceleration of the distal link
        var a_1 = [new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //acceleration of the proximal link
        var a_2 = [new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //acceleration of the distal link
        var Jp = new math.zeros(3, 3); //Jacobian used for velocity analysis
        var Q_2 = [new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3)]; //rotation matrix distal link
        var Q_1 = [new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3)]; //rotation matrix proximal link
        var I_2 = [new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3)]; //innertia matrix distal link
        var I_1 = [new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3), new math.zeros(3, 3)]; //innertia matrix proximal link
        var F_T = [new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //tangential force on the distal link
        var F_O = [new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //joint forces between the proximal and distal link
        var M_O = [new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //moments acting on actuator joint
        var F_ext = new math.zeros(6);
        var r_PS = new math.zeros(3);
        var c = [new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //arm from the actuator to the upper ball joints
        var b = [new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3), new math.zeros(3)]; //arm from the TCP to the lower ball joints
        var g = [0, 0, 9.81];
        var J_int = new math.zeros(6, 6);
    
        //Formula 3
        for (var i = 0; i < 3; i++) {
          ir._data[0][i] = -this.baseplateOffsetRadius + this.effectorOffsetRadius + effectorDynamics._data[0][0] * Math.cos(i * TWO_THIRD_PI) + effectorDynamics._data[1][0] * Math.sin(i * TWO_THIRD_PI); //x
          ir._data[1][i] = -effectorDynamics._data[0][0] * Math.sin(i * TWO_THIRD_PI) + effectorDynamics._data[1][0] * Math.cos(i * TWO_THIRD_PI); //y
          ir._data[2][i] = effectorDynamics._data[2][0]; //z
        }
    
        //Formula 5
        for (var i = 0; i < 3; i++) {
          phi[2]._data[i] = math.acos((-effectorDynamics._data[0][0] * Math.sin(i * TWO_THIRD_PI) + effectorDynamics._data[1][0] * Math.cos(i * TWO_THIRD_PI)) / this.lowerArmLength); //auxiliary angle phi3i
        }
    
        //Formula 7
        for (var i = 0; i < 3; i++) {
          phi[1]._data[i] = math.acos((Math.pow(ir._data[0][i], 2) + Math.pow(ir._data[1][i], 2) + Math.pow(ir._data[2][i], 2) - Math.pow(this.upperArmLength, 2) - Math.pow(this.lowerArmLength, 2)) / (2 * this.upperArmLength * this.lowerArmLength * Math.sin(phi[2]._data[i]))); //auxiliary angle phi2i
        }
    
        //Formula 9
        for (var i = 0; i < 3; i++) {
          phi[0]._data[i] = math.atan2(-(-this.upperArmLength * ir._data[2][i] - this.lowerArmLength * Math.sin(phi[2]._data[i]) * Math.cos(phi[1]._data[i]) * ir._data[2][i] + this.lowerArmLength * Math.sin(phi[2]._data[i]) * Math.sin(phi[1]._data[i]) * ir._data[0][i]), (this.upperArmLength * ir._data[0][i] + this.lowerArmLength * Math.sin(phi[2]._data[i]) * Math.sin(phi[1]._data[i]) * ir._data[2][i] + this.lowerArmLength * Math.sin(phi[2]._data[i]) * Math.cos(phi[1]._data[i]) * ir._data[0][i]));
          actuatorDynamics._data[i][0] = phi[0]._data[i];
        }
    
        //Formula 4
        for (var i = 0; i < 3; i++) {
          l_1[i]._data[0] = this.upperArmLength * Math.cos(phi[0]._data[i]);
          l_1[i]._data[1] = 0;
          l_1[i]._data[2] = this.upperArmLength * Math.sin(phi[0]._data[i]);
          l_1[i] = math.multiply(math.inv(T(i * TWO_THIRD_PI)), l_1[i]); //missing in paper, transofrmation to global coordinate system
    
          l_2[i]._data[0] = this.lowerArmLength * Math.sin(phi[2]._data[i]) * Math.cos(phi[0]._data[i] + phi[1]._data[i]);
          l_2[i]._data[1] = this.lowerArmLength * Math.cos(phi[2]._data[i]);
          l_2[i]._data[2] = this.lowerArmLength * Math.sin(phi[2]._data[i]) * Math.sin(phi[0]._data[i] + phi[1]._data[i]);
          l_2[i] = math.multiply(math.inv(T(i * TWO_THIRD_PI)), l_2[i]); //missing in paper, transofrmation to global coordinate system
        }
    
        //Formula 12
        for (var i = 0; i < 3; i++) {
          sA[i] = math.multiply(math.inv(T(i * TWO_THIRD_PI)), [0, 1, 0]);
        }
    
        //Formula 16
        for (var i = 0; i < 3; i++) {
          Jp._data[i][0] = l_2[i]._data[0];
          Jp._data[i][1] = l_2[i]._data[1];
          Jp._data[i][2] = l_2[i]._data[2];
        }
        Jp = math.multiply([[1 / (math.multiply(math.cross(sA[0], l_1[0]), l_2[0])), 0, 0], [0, 1 / (math.multiply(math.cross(sA[1], l_1[1]), l_2[1])), 0], [0, 0, 1 / (math.multiply(math.cross(sA[2], l_1[2]), l_2[2]))]], Jp); //all vectors in global coordinate system
    
    
        //Formula 15
        for (var i = 0; i < 3; i++) {
          actuatorDynamics._data[i][1] = math.squeeze(math.multiply(Jp, [[effectorDynamics._data[0][1]], [effectorDynamics._data[1][1]], [effectorDynamics._data[2][1]]])._data)[i]; //angular velocity in local axis
        }
    
        //Formula 18
        for (var i = 0; i < 3; i++) {
          omega_1[i] = math.multiply(actuatorDynamics._data[i][1], sA[i]); //angular velocity in global space
        }
    
        //Formula 20
        for (var i = 0; i < 3; i++) {
          omega_2[i] = math.divide(math.cross(l_2[i], math.subtract([effectorDynamics._data[0][1], effectorDynamics._data[1][1], effectorDynamics._data[2][1]], math.cross(omega_1[i], l_1[i]))), Math.pow(this.lowerArmLength, 2));
        }
    
        //Formula 24
        for (var i = 0; i < 3; i++) {
          actuatorDynamics._data[i][2] = math.dot(math.subtract(math.subtract([effectorDynamics._data[0][2], effectorDynamics._data[1][2], effectorDynamics._data[2][2]], math.cross(omega_1[i], math.cross(omega_1[i], l_1[i]))), math.cross(omega_2[i], math.cross(omega_2[i], l_2[i]))), l_2[i]) / math.multiply(math.cross(sA[i], l_1[i]), l_2[i]);
        }
    
        //Formula 23
        for (var i = 0; i < 3; i++) {
          alpha_1[i] = math.multiply(actuatorDynamics._data[i][2], sA[i]); //angular acceleration in global space
        }
    
        //Formula 25
        for (var i = 0; i < 3; i++) {
          alpha_2[i] = math.cross(math.divide(l_2[i], Math.pow(this.lowerArmLength, 2)), math.subtract(math.subtract(math.subtract([effectorDynamics._data[0][2], effectorDynamics._data[1][2], effectorDynamics._data[2][2]], math.cross(omega_1[i], math.cross(omega_1[i], l_1[i]))), math.cross(alpha_1[i], l_1[i])), math.cross(omega_2[i], math.cross(omega_2[i], l_2[i]))));
        }
    
        //Formula 26
        for (var i = 0; i < 3; i++) {
          a_1[i] = math.add(math.cross(alpha_1[i], math.divide(l_1[i], 2)), math.cross(omega_1[i], math.cross(omega_1[i], math.divide(l_1[i], 2))));
        }
    
        //Formula 27
        for (var i = 0; i < 3; i++) {
          a_2[i] = math.add(math.multiply(2, a_1[i]), math.add(math.cross(omega_2[i], math.cross(omega_2[i], math.divide(l_2[i], 2))), math.cross(alpha_2[i], math.divide(l_2[i], 2))));
        }
    
        //Formula 48
        for (var i = 0; i < 6; i++) {
          Q_2[i]._data[0][0] = math.divide(l_2[math.floor(i / 2)], this.lowerArmLength)._data[0];
          Q_2[i]._data[1][0] = math.divide(l_2[math.floor(i / 2)], this.lowerArmLength)._data[1];
          Q_2[i]._data[2][0] = math.divide(l_2[math.floor(i / 2)], this.lowerArmLength)._data[2];
    
          Q_2[i]._data[0][1] = math.divide(math.cross([1, 0, 0], l_2[math.floor(i / 2)]), math.norm(math.cross([1, 0, 0], l_2[math.floor(i / 2)])))._data[0];
          Q_2[i]._data[1][1] = math.divide(math.cross([1, 0, 0], l_2[math.floor(i / 2)]), math.norm(math.cross([1, 0, 0], l_2[math.floor(i / 2)])))._data[1];
          Q_2[i]._data[2][1] = math.divide(math.cross([1, 0, 0], l_2[math.floor(i / 2)]), math.norm(math.cross([1, 0, 0], l_2[math.floor(i / 2)])))._data[2];
    
          Q_2[i]._data[0][2] = math.divide(math.cross(l_2[math.floor(i / 2)], math.cross([1, 0, 0], l_2[math.floor(i / 2)])), math.norm(math.cross(l_2[math.floor(i / 2)], math.cross([1, 0, 0], l_2[math.floor(i / 2)]))))._data[0];
          Q_2[i]._data[1][2] = math.divide(math.cross(l_2[math.floor(i / 2)], math.cross([1, 0, 0], l_2[math.floor(i / 2)])), math.norm(math.cross(l_2[math.floor(i / 2)], math.cross([1, 0, 0], l_2[math.floor(i / 2)]))))._data[1];
          Q_2[i]._data[2][2] = math.divide(math.cross(l_2[math.floor(i / 2)], math.cross([1, 0, 0], l_2[math.floor(i / 2)])), math.norm(math.cross(l_2[math.floor(i / 2)], math.cross([1, 0, 0], l_2[math.floor(i / 2)]))))._data[2];
        }
    
        //Formula 50
        for (var i = 0; i < 3; i++) {
          Q_1[i] = math.multiply(math.inv(T(i * TWO_THIRD_PI)), math.matrix([[Math.cos(phi[0]._data[i]), 0, -Math.sin(phi[0]._data[i])], [0, 1, 0], [Math.sin(phi[0]._data[i]), 0, Math.cos(phi[0]._data[i])]]));
          I_1[i] = math.multiply(Q_1[i], math.multiply(this.upperArmInertiaMatrix, math.transpose(Q_1[i])));
        }
    
        //Formula 49
        for (var i = 0; i < 6; i++) {
          I_2[i] = math.multiply(Q_2[i], math.multiply(math.multiply(this.lowerArmMass, [[(Math.pow(this.lowerArmOuterRadius, 2) + Math.pow(this.lowerArmInnerRadius, 2)) / 2, 0, 0], [0, Math.pow(this.lowerArmLength, 2) / 12, 0], [0, 0, Math.pow(this.lowerArmLength, 2) / 12]]), math.transpose(Q_2[i])));
        }
    
        //Formula 47
        for (var i = 0; i < 6; i++) {
          F_T[i] = math.add(math.subtract(math.cross(math.divide(l_2[math.floor(i / 2)], 2), math.multiply(this.lowerArmMass, a_2[math.floor(i / 2)])), math.cross(math.divide(l_2[math.floor(i / 2)], 2), math.multiply(this.lowerArmMass, g))), math.add(math.multiply(I_2[i], alpha_2[math.floor(i / 2)]), math.cross(omega_2[math.floor(i / 2)], math.multiply(I_2[i], omega_2[math.floor(i / 2)]))));
        }
    
        //Formula 54
        r_PS._data[0] = this.effectorMassCenterPoint[0];
        r_PS._data[1] = -this.effectorMassCenterPoint[1];
        r_PS._data[2] = -this.effectorMassCenterPoint[2];
    
        for (var i = 0; i < 3; i++) { //calculate the position of the lower ball joints
          b[i * 2]._data[0] = this.effectorOffsetRadius;
          b[i * 2]._data[1] = -this.lowerArmOffset / 2;
          b[i * 2] = math.multiply(math.inv(T(i * TWO_THIRD_PI)), b[i * 2]);
          b[(i * 2) + 1]._data[0] = this.effectorOffsetRadius;
          b[(i * 2) + 1]._data[1] = this.lowerArmOffset / 2;
          b[(i * 2) + 1] = math.multiply(math.inv(T(i * TWO_THIRD_PI)), b[(i * 2) + 1]);
        }
    
        for (var i = 0; i < 3; i++) {
          F_ext._data[i] = this.effectorMass * (effectorDynamics._data[i][2] - g[i]) + F_T[0]._data[i] + F_T[1]._data[i] + F_T[2]._data[i] + F_T[3]._data[i] + F_T[4]._data[i] + F_T[5]._data[i];
          F_ext._data[i + 3] = math.cross(r_PS, math.multiply(this.effectorMass, math.subtract([effectorDynamics._data[0][2], effectorDynamics._data[1][2], effectorDynamics._data[2][2]], g)))._data[i];
          for (var j = 0; j < 6; j++) {
            F_ext._data[i + 3] = F_ext._data[i + 3] + math.cross(b[j], F_T[j])._data[i];
          }
        }
    
        //Formula 53
        for (var i = 0; i < 6; i++) {
          J_int._data[0][i] = math.divide(l_2[math.floor(i / 2)], this.lowerArmLength)._data[0];
          J_int._data[1][i] = math.divide(l_2[math.floor(i / 2)], this.lowerArmLength)._data[1];
          J_int._data[2][i] = math.divide(l_2[math.floor(i / 2)], this.lowerArmLength)._data[2];
    
          J_int._data[3][i] = math.cross(b[i], math.divide(l_2[math.floor(i / 2)], this.lowerArmLength))._data[0];
          J_int._data[4][i] = math.cross(b[i], math.divide(l_2[math.floor(i / 2)], this.lowerArmLength))._data[1];
          J_int._data[5][i] = math.cross(b[i], math.divide(l_2[math.floor(i / 2)], this.lowerArmLength))._data[2];
        }
    
        //Formula 52
        F_i = math.multiply(math.inv(J_int), F_ext);
    
        //Formula 55
        for (var i = 0; i < 6; i++) {
          F_O[i] = math.add(math.add(math.multiply(F_T[i], -1), math.multiply(F_i._data[i], math.divide(l_2[math.floor(i / 2)], this.lowerArmLength))), math.multiply(this.lowerArmMass, math.subtract(a_2[math.floor(i / 2)], g)));
        }
    
        //Formula 56
        for (var i = 0; i < 3; i++) {
          c[i * 2] = math.add(l_1[i], math.multiply(math.inv(T(i * TWO_THIRD_PI)), [0, -this.lowerArmOffset / 2, 0]));
          c[(i * 2) + 1] = math.add(l_1[i], math.multiply(math.inv(T(i * TWO_THIRD_PI)), [0, this.lowerArmOffset / 2, 0]));
        }
    
        for (var i = 0; i < 3; i++) {
          M_O[i] = math.add(M_O[i], math.add(math.cross(c[i * 2], F_O[i * 2]), math.cross(c[(i * 2) + 1], F_O[(i * 2) + 1])));
          M_O[i] = math.add(M_O[i], math.cross(math.divide(l_1[i], 2), math.multiply(this.upperArmMass, math.subtract(a_1[i], g))));
          M_O[i] = math.add(M_O[i], math.multiply(I_1[i], alpha_1[i]));
          M_O[i] = math.add(M_O[i], math.cross(omega_1[i], math.multiply(I_1[i], omega_1[i])));
        }
    
        //Formula 50 & 57
        for (var i = 0; i < 3; i++) {
          actuatorDynamics._data[i][3] = math.multiply(math.transpose(Q_1[i]), M_O[i])._data[1];
        }
    
        return actuatorDynamics;
    
        function T(a) {
          //helper function to ease transformation from local to global space
          var transform = new math.zeros(3, 3);
          transform._data[0][0] = Math.cos(a);
          transform._data[0][1] = Math.sin(a);
          transform._data[1][0] = -Math.sin(a);
          transform._data[1][1] = Math.cos(a);
          transform._data[2][2] = 1;
          return transform;
        }
      }
      getForcesLowerArm(externalForces) {
        /**
        * Computes the Jacobean used for calculating the effects of external forces on the distal links.
        * Multiplying the inverse of this matrix with the forces put on the platform results in the forces in the distal links.
        *
        * Procedure according to: Formula 52 and 53; J.Brinker, B.Corves and M.Wahle; A Comparative Study of Inverse Dynamics based on Clavel's Delta robot
        */
        var jf = new math.zeros(6, 6);
    
        for (var i = 0; i < 6; i++) {
          jf._data[0][i] = this.lowerArm[i].unit()[0];
          jf._data[1][i] = this.lowerArm[i].unit()[1];
          jf._data[2][i] = this.lowerArm[i].unit()[2];
    
          jf._data[3][i] = glm.cross((new Line(this.effectorPosition, this.lowerArm[i].end)).direction(), this.lowerArm[i].unit())[0];
          jf._data[4][i] = glm.cross((new Line(this.effectorPosition, this.lowerArm[i].end)).direction(), this.lowerArm[i].unit())[1];
          jf._data[5][i] = glm.cross((new Line(this.effectorPosition, this.lowerArm[i].end)).direction(), this.lowerArm[i].unit())[2];
        }
        return math.multiply(math.inv(jf), externalForces);
      }
      getEffectorForces() {
        /**
        * Calculates the force from the effector on the delta by default gravity is not enabled
        */
        var forces = new math.zeros(6);
        //forces
        forces._data[0] = -this.effectorMass * this.effectorAcceleration[0];
        forces._data[1] = -this.effectorMass * this.effectorAcceleration[1];
        forces._data[2] = -this.effectorMass * this.effectorAcceleration[2];
    
        //moments
        forces._data[3] = glm.cross(this.effectorMassCenterPoint, glm.mul(this.effectorAcceleration, -this.effectorMass))[0];
        forces._data[4] = glm.cross(this.effectorMassCenterPoint, glm.mul(this.effectorAcceleration, -this.effectorMass))[1];
        forces._data[5] = glm.cross(this.effectorMassCenterPoint, glm.mul(this.effectorAcceleration, -this.effectorMass))[2];
    
        return forces;
      }
      getEffectorDisplacement(externalForces) {
        /**
        * Computes the displacement of the end effector under the given external forces
        *
        * Procedure according to: M.Wahle and B.Corves; Stiffness Analysis of Clavel's DELTA Robot
        */
        //Formula 4
        var r = new math.zeros(3, 6);
        for (var i = 0; i < 3; i++) {
    
          r._data[0][i * 2] = glm.add(this.upperArm[i].direction(), this.lowerArm[i * 2].direction())[0];
          r._data[1][i * 2] = glm.add(this.upperArm[i].direction(), this.lowerArm[i * 2].direction())[1];
          r._data[2][i * 2] = glm.add(this.upperArm[i].direction(), this.lowerArm[i * 2].direction())[2];
    
          r._data[0][(i * 2) + 1] = glm.add(this.upperArm[i].direction(), this.lowerArm[(i * 2) + 1].direction())[0];
          r._data[1][(i * 2) + 1] = glm.add(this.upperArm[i].direction(), this.lowerArm[(i * 2) + 1].direction())[1];
          r._data[2][(i * 2) + 1] = glm.add(this.upperArm[i].direction(), this.lowerArm[(i * 2) + 1].direction())[2];
    
          /**
           * The following lines are added to ensure that the r_vector is from the centre of the ball joints
           * in the upper arm, to each ball joint in the upper arms. Refer to the document "Performance Benchmark"
           * showing the correct representation of the r_vector to be implemented in the section "VP Script Errors"
           */
          r._data[0][i * 2] = r._data[0][i * 2] - this.lowerArmOffset / 2 * -math.sin(i * TWO_THIRD_PI);
          r._data[1][i * 2] = r._data[1][i * 2] - this.lowerArmOffset / 2 * math.cos(i * TWO_THIRD_PI);
    
          r._data[0][(i * 2) + 1] = r._data[0][(i * 2) + 1] + this.lowerArmOffset / 2 * -math.sin(i * TWO_THIRD_PI);
          r._data[1][(i * 2) + 1] = r._data[1][(i * 2) + 1] + this.lowerArmOffset / 2 * math.cos(i * TWO_THIRD_PI);
    
        };
    
        //Formula 13, extend externalForces with zeros
        var fext = new math.zeros(24);
        for (var i = 0; i < 6; i++) {
          fext._data[i] = externalForces._data[i];
        };
    
        //Formula 14
        var jp = new math.zeros(24, 24);
        for (var i = 0; i < 6; i++) {
          jp._data[0][i] = this.lowerArm[i].unit()[0];
          jp._data[1][i] = this.lowerArm[i].unit()[1];
          jp._data[2][i] = this.lowerArm[i].unit()[2];
    
          jp._data[3][i] = glm.cross((new Line(this.effectorPosition, this.lowerArm[i].end)).direction(), this.lowerArm[i].unit())[0];
          jp._data[4][i] = glm.cross((new Line(this.effectorPosition, this.lowerArm[i].end)).direction(), this.lowerArm[i].unit())[1];
          jp._data[5][i] = glm.cross((new Line(this.effectorPosition, this.lowerArm[i].end)).direction(), this.lowerArm[i].unit())[2];
        }
        jp.setMatrix(6, 8, 0, 0, math.multiply(jp.getMatrix(0, 2, 0, 0), -1)); //-s1
        jp.setMatrix(6, 8, 1, 1, math.multiply(jp.getMatrix(0, 2, 1, 1), -1)); //-s1
        jp.setMatrix(9, 11, 2, 2, math.multiply(jp.getMatrix(0, 2, 2, 2), -1)); //-s2
        jp.setMatrix(9, 11, 3, 3, math.multiply(jp.getMatrix(0, 2, 3, 3), -1)); //-s2
        jp.setMatrix(12, 14, 4, 4, math.multiply(jp.getMatrix(0, 2, 4, 4), -1)); //-s3
        jp.setMatrix(12, 14, 5, 5, math.multiply(jp.getMatrix(0, 2, 5, 5), -1)); //-s3
    
        jp.setMatrix(15, 17, 0, 0, math.transpose(math.multiply(math.cross(r.getMatrix(0, 2, 0, 0), jp.getMatrix(0, 2, 0, 0)), -1))); //-(r1 x s1)T
        jp.setMatrix(15, 17, 1, 1, math.transpose(math.multiply(math.cross(r.getMatrix(0, 2, 1, 1), jp.getMatrix(0, 2, 1, 1)), -1))); //-(r2 x s1)T
    
        jp.setMatrix(18, 20, 2, 2, math.transpose(math.multiply(math.cross(r.getMatrix(0, 2, 2, 2), jp.getMatrix(0, 2, 2, 2)), -1))); //-(r3 x s2)T
        jp.setMatrix(18, 20, 3, 3, math.transpose(math.multiply(math.cross(r.getMatrix(0, 2, 3, 3), jp.getMatrix(0, 2, 3, 3)), -1))); //-(r4 x s2)T
    
        jp.setMatrix(21, 23, 4, 4, math.transpose(math.multiply(math.cross(r.getMatrix(0, 2, 4, 4), jp.getMatrix(0, 2, 4, 4)), -1))); //-(r5 x s3)T
        jp.setMatrix(21, 23, 5, 5, math.transpose(math.multiply(math.cross(r.getMatrix(0, 2, 5, 5), jp.getMatrix(0, 2, 5, 5)), -1))); //-(r6 x s3)T
        jp.setMatrix(6, 23, 6, 23, math.identity(18));
    
    
        //Formula 2, 15 and 16, seems to differ from implementation Bram in python. Implementation below according to paper.
        var tov = new math.identity(24, 24);
        var itov = new math.zeros(3, 3);
        for (var i = 0; i < 3; i++) {
          itov = math.multiply([[Math.cos(this.actuatorAngle[i]), 0, Math.sin(this.actuatorAngle[i])], [0, 1, 0], [-Math.sin(this.actuatorAngle[i]), 0, Math.cos(this.actuatorAngle[i])]], [[Math.cos(-TWO_THIRD_PI * i), -Math.sin(-TWO_THIRD_PI * i), 0], [Math.sin(-TWO_THIRD_PI * i), Math.cos(-TWO_THIRD_PI * i), 0], [0, 0, 1]]);
          tov.setMatrix((i * 3) + 6, (i * 3) + 8, (i * 3) + 6, (i * 3) + 8, math.matrix(itov));
          tov.setMatrix((i * 3) + 15, (i * 3) + 17, (i * 3) + 15, (i * 3) + 17, math.matrix(itov));
        };
    
    
        //Formula 17, Jacobean used for stiffness calculations
        var jpn = math.multiply(jp, math.inv(tov));
    
        //Stiffness matrix, according to 4.1
        var kdiag = math.identity(24, 24);
        kdiag = math.multiply(kdiag, Math.pow(10, 20));
    
        kdiag._data[0][0] = this.lowerArmExtensionalStiffness; //extension of the lower arm
        kdiag._data[1][1] = this.lowerArmExtensionalStiffness;
        kdiag._data[2][2] = this.lowerArmExtensionalStiffness;
        kdiag._data[3][3] = this.lowerArmExtensionalStiffness;
        kdiag._data[4][4] = this.lowerArmExtensionalStiffness;
        kdiag._data[5][5] = this.lowerArmExtensionalStiffness;
    
        kdiag._data[6][6] = this.upperArmExtensionalStiffness; //extension of the upper arm;
        kdiag._data[9][9] = this.upperArmExtensionalStiffness;
        kdiag._data[12][12] = this.upperArmExtensionalStiffness;
    
        kdiag._data[7][7] = this.upperArmHorizontalBendStiffness; //horizontal bend of the upper arm
        kdiag._data[10][10] = this.upperArmHorizontalBendStiffness;
        kdiag._data[13][13] = this.upperArmHorizontalBendStiffness;
    
        kdiag._data[8][8] = this.upperArmVerticalBendStiffness; //vertical bend of the upper arm
        kdiag._data[11][11] = this.upperArmVerticalBendStiffness;
        kdiag._data[14][14] = this.upperArmVerticalBendStiffness;
    
        kdiag._data[15][15] = 1 / (1 / this.gearboxTiltStiffness + 1 / this.upperArmTorsionalStiffness); //torsion around x-axis
        kdiag._data[18][18] = 1 / (1 / this.gearboxTiltStiffness + 1 / this.upperArmTorsionalStiffness);
        kdiag._data[21][21] = 1 / (1 / this.gearboxTiltStiffness + 1 / this.upperArmTorsionalStiffness);
    
        kdiag._data[16][16] = 1 / (1 / this.gearboxTorsionStiffness + 1 / (this.upperArmVerticalBendStiffness * math.pow(this.upperArmLength, 2))); //torsion around motor rotation axis (y-axis)
        kdiag._data[19][19] = 1 / (1 / this.gearboxTorsionStiffness + 1 / (this.upperArmVerticalBendStiffness * math.pow(this.upperArmLength, 2)));
        kdiag._data[22][22] = 1 / (1 / this.gearboxTorsionStiffness + 1 / (this.upperArmVerticalBendStiffness * math.pow(this.upperArmLength, 2)));
    
        kdiag._data[17][17] = 1 / (1 / this.gearboxTiltStiffness + 1 / (this.upperArmHorizontalBendStiffness * math.pow(this.upperArmLength, 2))); //torsion around z-axis
        kdiag._data[20][20] = 1 / (1 / this.gearboxTiltStiffness + 1 / (this.upperArmHorizontalBendStiffness * math.pow(this.upperArmLength, 2)));
        kdiag._data[23][23] = 1 / (1 / this.gearboxTiltStiffness + 1 / (this.upperArmHorizontalBendStiffness * math.pow(this.upperArmLength, 2)));
    
        var kp = math.multiply(math.multiply(jpn, kdiag), math.transpose(jpn));
    
        return math.resize(math.multiply(math.inv(kp), fext), [6]);
      }
      topCircleRadius() {
        var topConeZ = -((Math.sin(this.minArmAngle) * this.upperArmLength) + this.lowerArmLength); //top cut off (downwards from mount)
        var xOffset = (this.baseplateOffsetRadius - this.effectorOffsetRadius) + (this.upperArmLength * Math.cos(this.maxArmAngle)); //Sphere shape center
        var zOffset = -this.upperArmLength * Math.sin(Math.PI - this.maxArmAngle); //Sphere shape center
        var circleRadius = (new Sphere(glm.vec3(0, 0, zOffset), this.lowerArmLength)).intersect(new Plane(glm.vec3(0, 0, topConeZ), zAxis())).radius;
        return Math.sqrt(Math.pow(circleRadius, 2) - Math.pow(xOffset * Math.sin(ONE_THIRD_PI), 2)) - (xOffset / 2);
      }
}
