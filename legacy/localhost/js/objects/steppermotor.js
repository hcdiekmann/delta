class StepperMotor extends Actuator {
    constructor(position,rotation, scale) {
        super(position,rotation, scale, 0);
        this.name = this.name + this.id;    // to avoid having multiple steppermotors with the same name
        this.enable = false;
        this.speed = 1;
        this.currentRotation = 0;

        this.stepperMotorhash = "123456789";
        this.readCSV = false;
        this.readCSVCounter = 0;
    }

    create() {
        super.create();
        const size = 0.6;
        // box \/
        mPush();    // save original position
            noStroke();
            specularMaterial(120, 120, 120);
            mTranslate(0, 0, 0);
            mBox(this.id, size, size, size); 
        mPop();     // go back to original position
        // cylinder \/
        mPush();    // save origin position
            mTranslate(0, 0, 0.3);
            noStroke();
            specularMaterial(150,150,150);
            mRotateX(HALF_PI);
            cylinder(0.1, 0.5);
        mPop();     // go back to original position
        // cross \/
        mPush();    // save origin position
            noStroke();
            mTranslate(0, 0, 0.55);
            specularMaterial(150,150,150);
            mRotateZ(this.value);
            //mBox(this.id, 1, 0.2, 0.05);
            mBox(this.id, 0.2, 1, 0.05);
        mPop();     // go back to original position
    }

    update()
    {
        if(this.enable) // if servo enabled
        {
            // rotate the servo
            this.currentRotation += this.speed * DeltaTime();  // multiply the speed by 1/ framerate to make sure the speed is the same no matter what framerate is used

            this.SetValue(this.currentRotation % Math.PI)  // every rotation is a value between 0 and 3.14
            if (this.value < 0) // if it is negative, make it positive
            {
                this.SetValue(Math.PI + this.value);
            }
        }

        if(this.readCSV)
        {
            csvToArray((results)=>{
                if(results.data[0].hash === this.stepperMotorhash) {
                   this.currentRotation += this.speed * DeltaTime(); // multiply the speed by 1/ framerate to make sure the speed is the same no matter what framerate is used
                   this.value = this.currentRotation % results.data[this.readCSVCounter].pos0; // every rotation is a value between 0 and 3.14
                   this.readCSVCounter++;
                   if(results.data.length-1 === this.readCSVCounter) {
                       this.readCSVCounter = 0;
                   }
                }
                else {
                    console.log("verkeerde file!");
                }
            });
        }
    }

    setRotation(rotation) {
        this.value = rotation;
    }   
}