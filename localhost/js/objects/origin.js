class Origin extends DisplayObject
{
    constructor(position,rotation, scale) {
        super(position,rotation, scale);
    }

    create() {
        super.create(); // does a push, and translates to the correct world position
        noStroke();

        ambientMaterial(255);
        mBox(this.id, 0.05);

        ambientMaterial(0,96, 255); 
        mTorus(this.id,0.075,0.005,24,12); //Horizontal plane, Z-normal

        ambientMaterial(96,255, 0);
        mRotateX(PI/2);
        mTorus(this.id,0.075,0.005,24,12); //Vertical plane, Y-normal

        ambientMaterial(255,96,0);
        mRotateY(PI/2);
        mTorus(this.id,0.075,0.005,24,12); //Vertical plane, X-normal
    }
    
}