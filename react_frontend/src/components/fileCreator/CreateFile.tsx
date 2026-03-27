/* eslint-disable @typescript-eslint/no-unsafe-function-type */
import React from "react";
import { useState, useEffect, useRef } from "react";
import {
  Modal,
  ModalInputBox,
  ModalInputSelectIcon,
  ModalRow,
  ModalTitlebar,
  Entry,
  ModalInputSelectIconEntry,
  contrastSelector,
} from "jderobot-ide-interface";

import LoopRoundedIcon from "@mui/icons-material/LoopRounded";
import NotInterestedRoundedIcon from "@mui/icons-material/NotInterestedRounded";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { PythonIcon, CppIcon, RosIcon } from "Icons/index";
import { StyledTemplatesTitle } from "Styles/fileCreator/fileCreator.styles";

export interface newTemplate {
  fileType: string;
  fileName: string;
  templateType: string;
}

const initialNewFileModalData: newTemplate = {
  fileName: "",
  fileType: "plain",
  templateType: "empty",
};

const CreateAction = ({
  onSubmit,
  isOpen,
  onClose,
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  fileList,
  location,
  project,
}: {
  onSubmit: Function;
  isOpen: boolean;
  onClose: Function;
  fileList: Entry[];
  location: string;
  project: string;
}) => {
  const theme = useAcademyTheme();
  const focusInputRef = useRef<HTMLInputElement>(null);
  const [formState, setFormState] = useState(initialNewFileModalData);
  const [template, setTemplate] = useState<string>("empty");
  const [creationType, setCreationType] = useState<string>("plain");

  const textColor = contrastSelector(
    theme.palette.text,
    theme.palette.darkText,
    theme.palette.bg
  );

  ///////////////////////// TYPES ////////////////////////////////////////////////
  const plain: ModalInputSelectIconEntry = {
    id: "plain",
    title: "No template",
    iconType: "fill",
    icon: <NotInterestedRoundedIcon htmlColor={textColor} />,
  };

  const python: ModalInputSelectIconEntry = {
    id: "python",
    title: "Python",
    iconType: "fill",
    icon: <PythonIcon />,
  };

  const cpp: ModalInputSelectIconEntry = {
    id: "cpp",
    title: "C++",
    iconType: "fill",
    icon: <CppIcon />,
  };

  ///////////////////////// TEMPLATES //////////////////////////////////////////
  const pyReactive: ModalInputSelectIconEntry = {
    id: "py-reactive",
    title: "Reactive",
    iconType: "fill",
    icon: <LoopRoundedIcon htmlColor={textColor} />,
  };

  const pyRos: ModalInputSelectIconEntry = {
    id: "py-ros2",
    title: "ROS2 control",
    iconType: "fill",
    icon: <RosIcon />,
  };

  const cppReactive: ModalInputSelectIconEntry = {
    id: "cpp-reactive",
    title: "Reactive",
    iconType: "fill",
    icon: <LoopRoundedIcon htmlColor={textColor} />,
  };

  const cppRos: ModalInputSelectIconEntry = {
    id: "cpp-ros2",
    title: "ROS2 control",
    iconType: "fill",
    icon: <RosIcon />,
  };

  //////////////////////////////////////////////////////////////////////////////

  const onOptionTypeChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setCreationType(e.target.value);
    handleInputChange(e);
  };

  const onOptionTemplateChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setTemplate(e.target.value);
    handleInputChange(e);
  };

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current!.focus();
      }, 0);
    }
    setCreationType("plain");
    setTemplate("empty");
  }, [isOpen]);

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = event.target;

    if (name === "fileType" && value === "plain") {
      setFormState((prevFormData) => ({
        ...prevFormData,
        templateType: "empty",
      }));
    }

    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
  };

  const handleSubmit = (event: React.FormEvent) => {
    event.preventDefault();
    onSubmit(project, location, formState);
    setFormState(initialNewFileModalData);
    onClose();
  };

  const handleCancel = (event: React.FormEvent) => {
    if (event) {
      event.preventDefault();
    }

    onClose();
    setFormState(initialNewFileModalData);
  };

  const validPython =
    creationType === "python" && formState["fileName"].endsWith(".py");
  const validCpp =
    creationType === "cpp" && formState["fileName"].endsWith(".cpp");
  const validPlain = creationType === "plain" && formState.fileName !== "";
  const validName = validPython || validCpp || validPlain;

  const validPyTemplate =
    creationType === "python" && template.startsWith("py");
  const validCppTemplate = creationType === "cpp" && template.startsWith("cpp");
  const validTemplate = validPyTemplate || validCppTemplate;

  return (
    <Modal
      id="new-action-modal"
      isOpen={isOpen}
      onClose={handleCancel}
      onSubmit={handleSubmit}
      onReset={handleCancel}
    >
      <ModalTitlebar
        title="Create new file"
        htmlFor="fileName"
        hasClose
        handleClose={handleCancel}
      />
      <ModalRow type="input">
        <ModalInputBox
          isInputValid={validName}
          ref={focusInputRef}
          id="fileName"
          placeholder="File Name"
          onChange={handleInputChange}
          type="text"
          autoComplete="off"
          required
        />
      </ModalRow>
      <details>
        <StyledTemplatesTitle color={textColor}>
          Start from template
        </StyledTemplatesTitle>
        <ModalRow>
          <ModalInputSelectIcon
            id="fileType"
            title="Templates"
            onChange={onOptionTypeChange}
            selected={creationType}
            entries={[plain, python, cpp]}
          />
        </ModalRow>
        {creationType === "python" && (
          <ModalRow>
            <ModalInputSelectIcon
              id="templateType"
              title="Select Python template"
              onChange={onOptionTemplateChange}
              selected={template}
              entries={[pyReactive, pyRos]}
            />
          </ModalRow>
        )}
        {creationType === "cpp" && (
          <ModalRow>
            <ModalInputSelectIcon
              id="templateType"
              title="Select C++ template"
              onChange={onOptionTemplateChange}
              selected={template}
              entries={[cppReactive, cppRos]}
            />
          </ModalRow>
        )}
      </details>
      <ModalRow type="buttons">
        <button
          type="submit"
          id="create-new-action"
          disabled={!(validName && validTemplate)}
        >
          Create
        </button>
      </ModalRow>
    </Modal>
  );
};

export default CreateAction;
