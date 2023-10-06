class Robot extends DisplayObject
{
    constructor(name,position,rotation,scale)
    {
        super(position,rotation,scale);
        this.positions = [];
        this.name = name;
        this.actuatorList = [];
    }

    AddActuator(actuator)
    {
        actuatorList.push(actuator);
    }
}