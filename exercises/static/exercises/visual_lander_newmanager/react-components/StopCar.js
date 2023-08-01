import Button from '@mui/material/Button';

export const StopCar = () => {
    const handleClick = () => {
        window.RoboticsExerciseComponents.commsManager
          .send("#gui", {
            msg: "#stp",
          })
    }
    return (
        <Button variant="outlined" onClick={handleClick} sx={{color: "blue", borderColor: "blue"}}>Stop Car</Button>
    )
}