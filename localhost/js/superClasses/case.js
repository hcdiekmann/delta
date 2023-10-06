class Case extends DisplayObject{
    constructor(position,rotation, scale){
        const {x:x = 1, y:y = 1, z:z = 0.5} = scale;  // to set custom default value's for this class
        super (position, rotation,{x,y,z});
        this.thickness = 0.02;

        this.connectedProducts = [];
    }

    create() {
        super.create();
        mPush();
            mTranslate((this.scale.x + this.thickness) / 2,0,this.scale.z / 2 - this.thickness);
            box(this.thickness,this.scale.x,this.scale.z);
        mPop();
        mPush();
            mTranslate(-(this.scale.x + this.thickness) / 2,0,this.scale.z / 2 - this.thickness);
            box(this.thickness,this.scale.x,this.scale.z);
        mPop();
        mPush();
            mTranslate(0,(this.scale.y + this.thickness) / 2,this.scale.z / 2 - this.thickness);
            box(this.scale.y,this.thickness,this.scale.z);
        mPop();
        mPush();
            mTranslate(0,-(this.scale.y + this.thickness) / 2, this.scale.z / 2 - this.thickness);
            box(this.scale.y,this.thickness,this.scale.z);
        mPop();
        mPush();    // floor \/
            mTranslate(0,0,0);
            box(this.scale.y,this.scale.x,this.thickness);
        mPop();
    }

    connectChild(child)  // can be overloaded by other classes, like conveyors or boxes
    {
        super.connectChild(child);
        //console.log({z:this.GetZPos(child.scale),yrows:this.currentYRows(child.scale),maxz:this.maxZAmount(child.scale)});
        child.position.set(this.GetYPos(child.scale),this.GetXPos(child.scale),this.GetZPos(child.scale));  // set offset position to be in the box
        child.vectorRotation = false;
        this.connectedProducts.push(child);
    }
    // + ((this.connectedProducts.length) * child.scale.x)
    disconnectChild(child)  // can be overloaded by other classes, like conveyors or boxes
    {
        super.disconnectChild(child);
        const index = this.connectedProducts.indexOf(child); // remove child from connected list
        if (index > -1) {
            this.connectedProducts.splice(index, 1);
        }
    }

    GetXPos(scale)
    {
        return (this.scale.x * -0.5 + scale.x * 0.5) + this.currentXAmount(scale) * scale.x;
    }

    GetYPos(scale)
    {
        return (this.scale.y * -0.5 + scale.y * 0.5) + this.currentYAmount(scale) * scale.y;
    }

    GetZPos(scale)
    {
        return this.currentYRows(scale) * scale.z;
    }
    //(this.scale.z * -0.5 + scale.z * 0.5) + 
    
    currentXAmount(scale)
    {
        return this.connectedProducts.length - (this.maxXAmount(scale) * this.currentXRows(scale));
    }

    currentYAmount(scale)
    {
        return this.currentXRows(scale) - (this.maxYAmount(scale) * this.currentYRows(scale));
    }


    currentXRows(scale)
    {
        return Math.floor(this.connectedProducts.length / this.maxXAmount(scale));
    }

    currentYRows(scale)
    {
        return Math.floor(this.currentXRows(scale) / this.maxYAmount(scale));
    }

    currentZRows(scale)
    {
        return Math.floor(this.currentYRows(scale) / this.maxZAmount(scale));
    }


    maxXAmount(scale)
    {
        return Math.floor(this.scale.x / scale.x);
    }
    maxYAmount(scale)
    {
        return Math.floor(this.scale.y / scale.y);
    }
    maxZAmount(scale)
    {
        return Math.floor(this.scale.z / scale.z);
    }
}