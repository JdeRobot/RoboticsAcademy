import {
  createFile,
  createFolder,
  deleteFile,
  deleteFolder,
  getFile,
  getFileList,
  renameFile,
  renameFolder,
  uploadFile,
} from "Api";
import { CreateFile, newTemplate } from "Components/fileCreator";

export const fileExplorer = (warning: (msg: string) => void) => {
  return {
    name: "Code",
    list: (project: string) => {
      return getFileList(project);
    },
    file: {
      create: (project: string, location: string, name: string) => {
        return createFile(project, name, location);
      },
      get: (project: string, path: string) => {
        return getFile(project, path);
      },
      rename: (project: string, oldPath: string, newPath: string) => {
        return renameFile(project, oldPath, newPath);
      },
      delete: (project: string, path: string) => {
        return deleteFile(project, path);
      },
      upload: (
        project: string,
        path: string,
        name: string,
        content: string
      ) => {
        return uploadFile(project, name, path, content);
      },
    },
    folder: {
      create: (project: string, location: string, name: string) => {
        return createFolder(project, name, location);
      },
      rename: (project: string, oldPath: string, newPath: string) => {
        return renameFolder(project, oldPath, newPath);
      },
      delete: (project: string, path: string) => {
        return deleteFolder(project, path);
      },
    },
    modals: {
      createFile: {
        component: CreateFile,
        onCreate: (project: string, location: string, data: newTemplate) => {
          return createFile(
            project,
            data.fileName,
            location,
            data.templateType,
            warning
          );
        },
      },
    },
  };
};
