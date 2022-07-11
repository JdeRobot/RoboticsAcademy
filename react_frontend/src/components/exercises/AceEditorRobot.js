import * as React from 'react';
import {Box} from "@mui/material";
import AceEditor from "react-ace";

function AceEditorRobot() {
    const editorele_ = React.useRef(null);
    // const editorele = ace.edit("editor");
    // function setFontSize(augm) {
    //
	// 	  if (augm) {
	// 		if (editorele_.getFontSize() < 70)
	// 			editorele_.setFontSize(editorele_.getFontSize()+1);
	// 	  } else {
	// 		if (editorele_.getFontSize() > 2)
	// 			editorele_.setFontSize(editorele_.getFontSize()-1);
	// 	  }
	//   }

	  return (
          <Box sx={{m:3}} id="code-control">
              <Box sx={{display:"inline-flex"}} id="code_container">
                  <input type="button" id="aug_font" defaultValue={"+"} />
                  <input type="button" id="red_font"  defaultValue={"-"} />
                   <AceEditor
                id={"editor"}
                mode="python"
                theme="github"
                name="code"
                fontSize={14}
                showPrintMargin={true}
                showGutter={true}
                ref={ editorele_ }
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

              </Box>
              <div id="myModal" className="modal" tabIndex="-1" role="dialog">
                  <div className="modal-dialog" role="document">
                      <div className="modal-content">
                          <div className="modal-header">
                              <h5 className="modal-title">Enter a filename</h5>
                              <button type="button" className="close" data-dismiss="modal" aria-label="Close">
                                  <span aria-hidden="true">&times;</span>
                              </button>
                          </div>
                          <div className="modal-body">
                              <div className="md-form ml-0 mr-0">
                                  <input type="text" id="form29" className="form-control form-control-sm validate ml-0"/>
                              </div>
                          </div>
                          <div className="modal-footer">
                              <button type="button" className="btn btn-primary">Download
                              </button>
                              <button type="button" className="btn btn-secondary" data-dismiss="modal">Close</button>
                          </div>
                      </div>
                  </div>
              </div>
          </Box>
      );

}

export default AceEditorRobot;