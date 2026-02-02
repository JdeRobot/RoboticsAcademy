import { getFile, getHelperFile } from "Api";
import { Entry } from "jderobot-ide-interface";
import JSZip from "jszip";

export const saveCode = (
  fileName: string,
  python_code: string,
  language?: string
) => {
  let extension = ".py";
  if (language === "cpp") {
    extension = ".cpp";
  }
  const blob = new Blob([python_code], { type: "text/plain; charset=utf-8" });
  const a = document.createElement("a"),
    url = URL.createObjectURL(blob);
  a.href = url;
  a.download = fileName + extension;
  document.body.appendChild(a);
  a.click();
  setTimeout(function () {
    document.body.removeChild(a);
    window.URL.revokeObjectURL(url);
  }, 0);
};

export function subscribe(eventName: string, listener: (e: unknown) => void) {
  document.addEventListener(eventName, listener);
}

export function unsubscribe(eventName: string, listener: () => void) {
  document.removeEventListener(eventName, listener);
}

export function publish(eventName: string, extra?: unknown) {
  const event = new CustomEvent(eventName, { detail: extra });
  document.dispatchEvent(event);
}

export const zipHelperFiles = async (
  zip: JSZip,
  files: Entry[],
  project: string,
  language: string,
  entrypoint: Entry
) => {
  for (const file of files) {
    if (file.is_dir) {
      await zipHelperFolder(zip, file, project, language, entrypoint);
    } else {
      await zipHelperFile(
        zip,
        file.path,
        file.name,
        project,
        language,
        entrypoint
      );
    }
  }
};

const zipHelperFile = async (
  zip: JSZip,
  file_path: string,
  file_name: string,
  project: string,
  language: string,
  entrypoint: Entry
) => {
  let content = await getHelperFile(project, language, file_path);

  if (language === "cpp") {
    content = content.replace("academy.cpp", `${entrypoint.path}`);
  }

  zip.file(file_name, content);
};

const zipHelperFolder = async (
  zip: JSZip,
  file: Entry,
  project: string,
  language: string,
  entrypoint: Entry
) => {
  const folder = zip.folder(file.name);

  if (folder === null) {
    return;
  }

  for (let index = 0; index < file.files.length; index++) {
    const element = file.files[index];
    if (element.is_dir) {
      await zipHelperFolder(folder, element, project, language, entrypoint);
    } else {
      await zipHelperFile(
        folder,
        element.path,
        element.name,
        project,
        language,
        entrypoint
      );
    }
  }
};

export const zipCodeFiles = async (
  zip: JSZip,
  files: Entry[],
  project: string
) => {
  for (const file of files) {
    if (file.is_dir) {
      await zipCodeFolder(zip, file, project);
    } else {
      await zipCodeFile(zip, file, project);
    }
  }
};

const zipCodeFile = async (zip: JSZip, file: Entry, project: string) => {
  const content = await getFile(project, file.path, file.binary);
  zip.file(file.name, content, { binary: file.binary });
};

const zipCodeFolder = async (zip: JSZip, file: Entry, project: string) => {
  const folder = zip.folder(file.name);

  if (folder === null) {
    return;
  }

  for (let index = 0; index < file.files.length; index++) {
    const element = file.files[index];
    if (element.is_dir) {
      await zipCodeFolder(folder, element, project);
    } else {
      await zipCodeFile(folder, element, project);
    }
  }
};
