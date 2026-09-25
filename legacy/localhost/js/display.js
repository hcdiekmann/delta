class Display{
    constructor()
    {
        this.Initialized = false;
        this.clickableObjects = 0;	// to store all clickable objects, and give each of them a unique id
        this.clickableObjectCallback = new Array(1);

        this.cameraPosition = glm.vec3(2.0,3.0,1.0);
        this.cameraTarget = glm.vec3(0,0,0);
        this.cameraUp = glm.vec3(0,0,-1);

        this.objectArray = [];   // objects that need its update or draw method called every draw, go in this array
        this.deltaRobotArray = []; // objects that are deltarobots go in this array also
    }

    addObject(object)
    {
        if(object instanceof DeltaRobot)
        {
            if (object.position.x == 0 && object.position.y == 0 && object.position.z == 0) // only control the delta robot's position if it is not changed
            {
                if (this.deltaRobotArray.length > 0)
                {
                    const _radius = this.deltaRobotArray[this.deltaRobotArray.length - 1].topCircleRadius() + object.topCircleRadius() + 0.5;
                    object.position.x = this.deltaRobotArray[this.deltaRobotArray.length - 1].position.x + _radius;
                    object.position.y = this.deltaRobotArray[0].position.y; // all robots should have the same y
                }
                else
                {
                    // make sure first robot doesnt clip with origin
                    object.position.y = -(object.topCircleRadius() + 0.5);
                }
            }
            
            this.deltaRobotArray.push(object);
        }
        this.objectArray.push(object);
    }

    initialise()
    {
        mCreateCanvas(windowWidth-540, windowHeight, WEBGL);
        this.Initialised = true;
    }

    drawObjects()
    {
        if (!this.Initialised) this.initialise();
        mResetMatrix();
        // set camera positions \/\/
        mPerspective(60 * PI/180, width/height, 0.01, 50000);
        mCamera(this.cameraPosition[0],this.cameraPosition[1],this.cameraPosition[2],this.cameraTarget[0],this.cameraTarget[1],this.cameraTarget[2], this.cameraUp[0], this.cameraUp[1], this.cameraUp[2]);
        // adjust environment \/\/
        mBackground(192);
        ambientLight(200);
        pointLight(63, 63, 63, 0, 0, 1000);

        // display objects \/\/
        this.objectArray.forEach(element => {    // call the update and display function for each object
            if(typeof element.update === 'function')
            {
                element.update();
            }
            if(typeof element.create === 'function')
            {
                element.create();
                element.reset();    // so that mPop() is called
            }
        });
    }
}

let count = 0;
let productspeed = 200;

function draw() {   // called every frame
    display.drawObjects();


    // for testing the products
    //if (count % productspeed === 0) addProduct();
    //count++;
}

// returns: the time between each frame, used to make sure speeds are the same on each framerate
function DeltaTime()
{
    let toReturn = (1/ frameRate());
    if (toReturn === Infinity) { // avoid returning an infinite deltatime
        toReturn = (1/60); // use standard framerate of 60 FPS
    }
    return toReturn;
}