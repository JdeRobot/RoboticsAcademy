import React, {Fragment} from "react";
import './css/TestShowScreen.css';
import classNames from "classnames";

const TestShowScreen = (props) => {
    const [image, setImage] = React.useState(null);
    const [code, setCode] = React.useState("");

    const classes = classNames({
        "test-show-screen": true,
    });

    React.useEffect(() => {
        console.log("TestShowScreen subscribing to ['update'] events");
        setImage("https://via.placeholder.com/800x600.png?text=No%20image%20received%20from%20exercise");

        const callback = (message) => {
            const update = message.data.update;
            if (update.image) {
                const image = JSON.parse(update.image);
                setImage(`data:image/png;base64,${image.image}`);
            } else {
                setImage("https://via.placeholder.com/800x600.png?text=No%20image%20received%20from%20exercise");
            }
        };

        RoboticsExerciseComponents.commsManager.subscribe([
                RoboticsExerciseComponents.commsManager.events.UPDATE
            ],
            callback);

        return () => {
            console.log("TestShowScreen unsubscribing from ['state-changed'] events");
            RoboticsExerciseComponents.commsManager.unsubscribe([
                    RoboticsExerciseComponents.commsManager.events.UPDATE
                ],
                callback);
        }
    }, []);

    const changeCode = (event) => {
        setCode(event.target.value);
    };

    const sendCode = (event) => {
        RoboticsExerciseComponents.commsManager.send("load", {
            code: code
        }).then(message => {
            console.log("code loaded");
        }).catch(response => {
            console.error(response);
        })
    };

    return (
        <div className={"panel-parent"}>
            <div className={"panel"}>
                <textarea value={code} onChange={changeCode} cols={80}/>
                <div className={classes} onClick={sendCode}>Load code</div>
            </div>
            <div className={"panel"}>
                <img src={image} alt={"Exercise screen"} className={classes}/>
            </div>
        </div>
    );
};

export default TestShowScreen;