import editor from "react-ace";
import * as React from 'react';
import {Box} from "@mui/material";
import AceEditor from "react-ace";

function AceEditorRobot() {
    // const editorele = ace.edit("editor");
    function setFontSize(augm) {
		  // if (augm) {
			// if (editorele.getFontSize() < 70)
			// 	editorele.setFontSize(editorele.getFontSize()+1);
		  // } else {
			// if (editorele.getFontSize() > 2)
			// 	editorele.setFontSize(editorele.getFontSize()-1);
		  // }
	  }
    return (
        <div sx={{m:3}}>
            <div id={"code-control"}>
                <div id={"code_container"}>
                    <input type="button" id="aug_font" onClick={setFontSize(true)} value="+"/>
                    <input type="button" id="red_font" onClick={setFontSize(false)} value="-"/>
            <AceEditor
                id={"editor"}
                mode="python"
                theme="github"
                name="blah2"
                fontSize={14}
                showPrintMargin={true}
                showGutter={true}
                highlightActiveLine={true}
                value={`from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`}
                setOptions={{
                    enableBasicAutocompletion: true,
                    enableLiveAutocompletion: true,
                    enableSnippets: true,
                    showLineNumbers: true,
                    tabSize: 2,
                }}/>
        </div>
                </div>
            </div>
    );
}

export default AceEditorRobot;