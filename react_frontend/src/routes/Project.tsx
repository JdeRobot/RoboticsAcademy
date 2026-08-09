import React, { useEffect, useRef } from "react";
import { Params, useLoaderData } from "react-router";
import { lazy, Suspense } from "react";
import WebGUIPreview from "Components/visualizers/WebGUIPreview";
import ExerciseContainer from "Components/layouts/ExerciseContainer";
import { exitProject, getProjectData } from "Api";
import WebGUILoading from "Components/visualizers/WebGUILoading";
import { ExerciseData } from "Types/exercises";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { BasicHeader } from "Components/headers";
import {
  StyledErrorContainer,
  StyledErrorMessage,
} from "Styles/pages/Project.styles";

export const loader = async ({
  params,
}: {
  params: Params<string>;
}): Promise<Omit<ExerciseData, "worlds">> => {
  return await getProjectData(params.proj_id);
};

const Exercise = () => {
  const hasRender = useRef(false);
  const data = useLoaderData<Omit<ExerciseData, "worlds">>();

  const WebGui = lazy(async () => {
    return import(`exercises/${data.exercise_id}/frontend/WebGUI.tsx`).catch(
      (error) => {
        console.error("Component Failed Loading:", error);
        return { default: WebGUIPreview };
      },
    );
  });

  useEffect(() => {
    window.addEventListener("beforeunload", () => exitProject());

    return () => {
      window.removeEventListener("beforeunload", () => exitProject());
      // This is to fix this: fires twice thanks to react Strict mode
      if (hasRender.current || process.env.NODE_ENV !== "development") {
        exitProject();
      }
      hasRender.current = true;
    };
  }, []);

  // TODO: only tmp
  const additionalEntrypoints = data.tags.includes("MULTI-ENTRYPOINT") ? ["/resources/exercises/drone_cat_mouse/mouse.py"] : undefined

  return (
    <>
      <ExerciseContainer
        multiLanguage={data.tags.includes("MULTILANGUAGE")}
        additionalEntrypoints={additionalEntrypoints}
        project={data.exercise_id}
        name={data.name}
        tools={data.tools}
        url={data.url !== "" ? data.url : undefined}
      >
        <Suspense fallback={<WebGUILoading />}>
          <WebGui />
        </Suspense>
      </ExerciseContainer>
    </>
  );
};

export const ErrorBoundary = () => {
  const theme = useAcademyTheme();

  return (
    <>
      <BasicHeader />
      <StyledErrorContainer bgColor={theme.palette.bg}>
        <StyledErrorMessage color={theme.palette.error}>
          You have open a Robotics Academy exercise in another tab or window
        </StyledErrorMessage>
        <StyledErrorMessage color={theme.palette.error}>
          You can only have one open at a time
        </StyledErrorMessage>
        <StyledErrorMessage color={theme.palette.error}>
          Close the other instance or exit the exercise and refresh the page to
          continue
        </StyledErrorMessage>
      </StyledErrorContainer>
    </>
  );
};

export default Exercise;
