import React from 'react';
import '../styles/Exercise.css';
import RoboticsTheme from './RoboticsTheme';

function Exercise() {

     function onClickTheory() {
         if(!theoryMode){
             setTheoryMode(true);
             setCodeMode(false);
             setForumMode(false);
         }
     }
     function onClickCode() {
         if(!codeMode){
             setTheoryMode(false);
             setCodeMode(true);
             setForumMode(false);
         }
     }
     function onClickForum() {
         if(!forumMode){
             setTheoryMode(false);
             setCodeMode(false);
             setForumMode(true);
         }
     }
     function getLaunchLevel() {
         return launchLevel;
     }
  return (
    <RoboticsTheme>
      <div className="Exercise">
      </div>
    </RoboticsTheme>
  );
}

export default Exercise;