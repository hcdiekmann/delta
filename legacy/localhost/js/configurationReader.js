

function LoadConfiguration(configuration)
{
    configuration.forEach(element => {
        createObject(element.type,element.pos,element.rot,element.scale,element.additionalConfig)
    });
}

function createObject(type = "", pos = {}, rot = {}, scale = {}, additionalConfig = {})
{
    let createdObject = null;
    switch (type) {
        case "deltaRobot":
            createdObject = new DeltaRobot(pos,rot,scale,additionalConfig);
        break;  
        case "conveyor":
            createdObject = new Conveyor(pos,rot,scale);
        break;
        case "case":
            createdObject = new Case(pos,rot,scale);
        break;
        case "product":
            createdObject = new Product(pos,rot,scale);
        break;
        case "stepperMotor":
            createdObject = new StepperMotor(pos,rot,scale);
        break;
        case "":
            console.log("type string is empty!!");
        break;
        default:
            break;
    }
    display.addObject(createdObject);

    return createdObject;
}
