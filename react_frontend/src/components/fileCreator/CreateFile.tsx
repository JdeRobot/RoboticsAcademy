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
import FileDownloadRoundedIcon from "@mui/icons-material/FileDownloadRounded";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { PythonIcon, CppIcon } from "Icons/index";

export interface newTemplate {
  fileName: string;
  templateType: string;
}

const initialNewFileModalData: newTemplate = {
  fileName: "",
  templateType: "empty",
};

const CreateAction = ({
  onSubmit,
  isOpen,
  onClose,
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
    title: "Plain File",
    iconType: "fill",
    icon: <FileDownloadRoundedIcon htmlColor={textColor} />,
  };

  const python: ModalInputSelectIconEntry = {
    id: "python",
    title: "Python",
    iconType: "fill",
    icon: <PythonIcon htmlColor={textColor} />,
  };

  const cpp: ModalInputSelectIconEntry = {
    id: "cpp",
    title: "C++",
    iconType: "fill",
    icon: <CppIcon htmlColor={textColor} />,
  };

  ///////////////////////// TEMPLATES //////////////////////////////////////////
  const pyReactive: ModalInputSelectIconEntry = {
    id: "py-reactive",
    title: "Reactive",
    iconType: "fill",
    icon: <FileDownloadRoundedIcon htmlColor={textColor} />,
  };

  const pyRos: ModalInputSelectIconEntry = {
    id: "py-ros2",
    title: "ROS2 control",
    iconType: "fill",
    icon: <FileDownloadRoundedIcon htmlColor={textColor} />,
  };

  const cppReactive: ModalInputSelectIconEntry = {
    id: "cpp-reactive",
    title: "Reactive",
    iconType: "fill",
    icon: <FileDownloadRoundedIcon htmlColor={textColor} />,
  };

  const cppRos: ModalInputSelectIconEntry = {
    id: "cpp-ros2",
    title: "ROS2 control",
    iconType: "fill",
    icon: <FileDownloadRoundedIcon htmlColor={textColor} />,
  };

  //////////////////////////////////////////////////////////////////////////////

  const onOptionTypeChange = (e: any) => {
    setCreationType(e.target.value);
    handleInputChange(e);
  };

  const onOptionTemplateChange = (e: any) => {
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

  const handleInputChange = (event: any) => {
    const { name, value } = event.target;

    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
  };

  const handleSubmit = (event: any) => {
    event.preventDefault();
    onSubmit(project, location, formState);
    setFormState(initialNewFileModalData);
    onClose();
  };

  const handleCancel = (event: any) => {
    if (event) {
      event.preventDefault();
    }

    onClose();
    setFormState(initialNewFileModalData);
  };

  return (
    <Modal
      id="new-action-modal"
      isOpen={isOpen}
      onClose={handleCancel}
      onSubmit={handleSubmit}
      onReset={handleCancel}
    >
      <ModalTitlebar
        title="Load template"
        htmlFor="fileName"
        hasClose
        handleClose={handleCancel}
      />
      <ModalRow type="input">
        <ModalInputBox
          isInputValid={formState.fileName === ""}
          ref={focusInputRef as any}
          id="fileName"
          placeholder="File Name"
          onChange={handleInputChange}
          type="text"
          autoComplete="off"
          required
        />
      </ModalRow>
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
            title="Select Template Type"
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
            title="Select Template Type"
            onChange={onOptionTemplateChange}
            selected={template}
            entries={[cppReactive, cppRos]}
          />
        </ModalRow>
      )}
      <ModalRow type="buttons">
        <button type="submit" id="create-new-action">
          Create
        </button>
      </ModalRow>
    </Modal>
  );
};

export default CreateAction;
