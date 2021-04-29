function getYaw(qw,qx,qy,qz) {
       var rotateZa0=2.0*(qx*qy + qw*qz);
       var rotateZa1=qw*qw + qx*qx - qy*qy - qz*qz;
       var rotateZ=0.0;
       if(rotateZa0 != 0.0 && rotateZa1 != 0.0){
           rotateZ=Math.atan2(rotateZa0,rotateZa1);
       }
       return rotateZ;
}

function getRoll(qw,qx,qy,qz){
       rotateXa0=2.0*(qy*qz + qw*qx);
       rotateXa1=qw*qw - qx*qx - qy*qy + qz*qz;
       rotateX=0.0;

       if(rotateXa0 != 0.0 && rotateXa1 !=0.0){
           rotateX=Math.atan2(rotateXa0, rotateXa1);
       }
       return rotateX;
}
function getPitch(qw,qx,qy,qz){
       rotateYa0=-2.0*(qx*qz - qw*qy);
       rotateY=0.0;
       if(rotateYa0>=1.0){
           rotateY=Math.PI/2.0;
       } else if(rotateYa0<=-1.0){
           rotateY=-Math.PI/2.0
       } else {
           rotateY=Math.asin(rotateYa0)
       }

       return rotateY;
}
