let productAmount = 0;  // to store the total amount of product, to assign each with its own unique id

class DisplayObject
{
    constructor(position, rotation, scale) {
        let {x: xPos = 0, y: yPos = 0, z: zPos = 0, cPos: cPos = 0} = position;      // get position value's from parameter object, default = 0
        let {x: xRot = 0, y: yRot = 0, z: zRot = 0, vector: vectordirection = false} = rotation;      // get rotation value's from parameter object, default = 0
        let {x: xScale = 0.5, y: yScale = 0.5, z: zScale = 0.5} = scale;
        xPos ||= 0; yPos ||= 0; zPos ||= 0;     // make sure position is not null or undefined
        xRot ||= 0; yRot ||= 0; zRot ||= 0;     // make sure rotation is not null or undefined
        xScale ||= 0; yScale ||= 0; zScale ||= 0;     // make sure rotation is not null or undefined

        this.id = productAmount;
        productAmount++;                            // to assign every Object with its own ID

        this.position = new p5.Vector(xPos,yPos,zPos);     // at what position this object should be placed
        this.rotation = new p5.Vector(xRot,yRot,zRot);     // at what rotation this object should be placed
        this.scale = new p5.Vector(xScale, yScale, zScale);// what scale this object has

        this.vectorRotation = vectordirection;
        if (this.vectorRotation) this.rotation.normalize();
    
        this.parent = null;         // if this is set to a different object, this object's position will be an offset of the parent's position

        this.ConveyorPosition = cPos;   // when connected to a conveyor, this will be used to set its position in the width of the conveyor
        this.startValue = 0;            // when connected to a conveyor, this will be used to set its position in the length of the conveyor
    }

    // connects to a parent, this object's position will be an offset of its parent's position
    Connect(parent) {
        if (this.parent != null) this.parent.disconnectChild(this); // first disconnect from previous parent
        this.parent = parent;
        this.parent.connectChild(this);
    }

    // disconnects from its parent
    Disconnect() {
        this.parent.disconnectChild(this);
        this.parent = null;
    }

    connectChild(child)  // can be overloaded by other classes, like conveyors or boxes
    {
    }

    disconnectChild(child)  // can be overloaded by other classes, like conveyors or boxes
    {
    }

    update() //abstract
    {
    }

    /// automatically sets the cursor to the correct world position
    create()
    {
        mPush();
        this.translateCursor();         // move the cursor so the model is placed at the correct position
        this.rotateCursor(false)        // should already be rotated to parents rotation
    }

    // resets any changes made, called after the create() method finished
    reset() 
    {
        mPop();
    }

    

    // translates the p5 cursor to the correct position, while keeping the parent's position and rotation into account
    translateCursor()
    {
        if (this.parent != null)
        {
            this.parent.translateCursor();      // first translate to the parent's position
            this.parent.rotateCursor(false);    // rotate to the parent's rotation, while not taking the parent of this parent into account, as this method will also handle that rotation
        }
        mTranslate(this.position.x,this.position.y,this.position.z);
    }


    // rotates the p5 cursor to the correct rotation, while keeping the parent's rotation into account, and checking if it is a vector rotation
    rotateCursor(TakeParentIntoAccount = true)
    {
        if (this.parent != null && TakeParentIntoAccount) {
            this.parent.rotateCursor();
        }
        if (this.vectorRotation) {  // set correct rotation
            let vectorGround = createVector(this.rotation.x,this.rotation.y);
            let angleZ = -vectorGround.angleBetween(createVector(1,0));   //assume it is currently pointing in the Y direction
            angleZ ||= 0;
            mRotateZ(angleZ);

            let vectorHeight = createVector(sqrt(vectorGround.x * vectorGround.x) + (vectorGround.y * vectorGround.y), this.rotation.z);
            let angleY = vectorHeight.angleBetween(createVector(1,0));   //assume it is currently pointing in the X direction
            angleY ||= 0;
            mRotateY(angleY);
        }
        else {
            mRotateX(this.rotation.x);
            mRotateY(this.rotation.y);
            mRotateZ(this.rotation.z);
        }
    }
}