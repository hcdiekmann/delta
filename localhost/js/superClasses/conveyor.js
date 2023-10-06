class Conveyor extends DisplayObject{
    constructor (position, rotation, scale)
    {
        const {x:xScale = 1, y:yScale = 2, z:zScale = 0.05} = scale;  // to set custom default value's for this class
        super (position, rotation, {x:xScale, y:yScale, z:zScale});
        this.length = scale.y;
        this.scaling = 0.1;             // to scale the actuator value to world position
        this.speed = 3;
        this.nextParent = null;         // reference to what object the products will connect to after they reach the end of the conveyor

        this.connectedObjects = [];
        this.actuator = new Actuator({},{},{});
    }

    connectChild(child)
    {
        super.connectChild(child);
        child.position.set(0,0,0);  // reset offset position
        child.startValue = this.actuator.value;
        this.connectedObjects.push(child);
    }

    disconnectChild(child)  {   
        const index = this.connectedObjects.indexOf(child); // remove child from connected list
        if (index > -1) {
            this.connectedObjects.splice(index, 1);
        }
    }

    update()
    {
        super.update();
        this.actuator.value += this.speed * DeltaTime();

        // set the correct position of the object \/
        this.connectedObjects.forEach((object) => {
            if (!(object instanceof DisplayObject)) return; // cancel if this is not a DisplayObject
            object.position.set((this.actuator.value- object.startValue) * this.scaling,object.ConveyorPosition,0);    // move to the correct distance

            if ((this.actuator.value- object.startValue) * this.scaling >= this.length){  // if this object reached the end
                if (this.nextParent != null){    // connect to the next parent, if there is one
                    object.Connect(this.nextParent);
                    
                }else {
                    object.Disconnect();
                }
                return;
            }
        });
    }

    create()
    {
        super.create();
        noStroke();
        specularMaterial(0,0,0);
        mTranslate(this.length / 2,0,this.scale.z / -2);     // to have the conveyor's start position be the start of the box
        box(this.length,this.scale.x,this.scale.z);       // create conveyor
    }
}