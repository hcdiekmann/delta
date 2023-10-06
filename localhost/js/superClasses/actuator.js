class Actuator extends DisplayObject
{
    constructor(position,rotation,scale)
    {
        super(position,rotation,scale);
        this.value = 0;
        this.interpolateTime = 0;
        this.targetValue = 0;
        this.startValue = 0;
        this.starttime = Date.now();
        this.reachedCallback = null;
        this.originalObject = null;
    }

    // targetvalue:     value the actuator will move to
    // interpolateTime: time in milliseconds it will take for the actuator to get to the target value
    SetValue(_targetValue, _interpolateTime = 0, _reachedCallback = null, _originalObject = null)
    {
        this.originalObject = _originalObject;
        this.reachedCallback = _reachedCallback;
        this.interpolateTime = Number(_interpolateTime);
        if (_interpolateTime <= 0) {
            this.value = _targetValue;
            if (this.reachedCallback != null) this.reachedCallback(this.originalObject);
            return;
        }
        this.targetValue = Number(_targetValue);
        this.starttime = Date.now();
        this.startValue = Number(this.value);
    }

    update()
    {
        super.update();

        if (this.interpolateTime > 0) {    // if actuator should interpolate
            const timeSinceStart = Date.now() - this.starttime;
            // Move to value between start value and target value
            this.value = this.startValue + ((timeSinceStart / this.interpolateTime) * (this.targetValue - this.startValue));    
            if (timeSinceStart >= this.interpolateTime) {   // if should have reached target value
                this.value = this.targetValue;
                this.interpolateTime = 0;
                if (this.reachedCallback != null) this.reachedCallback(this.originalObject);
            }
        }
    }
}