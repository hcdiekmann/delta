class Product extends DisplayObject
{
    constructor(position,rotation, scale)
    {
        const {x:x = 0.1, y:y = 0.2, z:z = 0.05} = scale;  // to set custom default value's for this class
        super(position,rotation, {x,y,z});
    }

    create()
    {
        super.create();
        mPush();
            specularMaterial(255,0,0);
            mTranslate(0,0,this.scale.z / 2);
            box(this.scale.y,this.scale.x,this.scale.z);
        mPop();
    }
}
