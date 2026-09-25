class Servo extends Actuator
{
    constructor (position,rotation,scale)
    {
        super (position,rotation,scale);
    }

    create()
    {
        super.create();
        mPush();
            //reductor
            noStroke();
            ambientMaterial(127);
            specularMaterial(255);
            mTranslate(0,.0835,0);
            mCylinder(this.id,0.059, 0.007);
            mTranslate(0,0.0215,0);
            mCylinder(this.id,0.0475, 0.036);
            mTranslate(0,0.0285,0);
            mCone(this.id,0.0475,0.021);
            mTranslate(0,-0.001,0);
            mCylinder(this.id,0.034,0.019);
            mTranslate(0,0.0225,0);
            mBox(this.id,0.082,0.026,0.082);

            //servo
            ambientMaterial(32);
            specularMaterial(32);
            mTranslate(0,0.0165,0);
            mBox(this.id,0.082,0.007,0.082);
            mTranslate(0,0.0705,0);
            mBox(this.id,0.082,0.134,0.054);
            mBox(this.id,0.054,0.134,0.082);
            mTranslate(0,0.083,0);
            mCylinder(this.id,0.035,0.032);
            mTranslate(0,0.02125,0);
            mCylinder(this.id,0.030,0.0105);
        mPop();
    }
}