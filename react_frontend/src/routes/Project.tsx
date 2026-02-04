import React, { useEffect } from "react";
import { Params, useLoaderData } from "react-router-dom";
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
}): Promise<Omit<ExerciseData, "universes">> => {
  return await getProjectData(params.proj_id);
};

const Exercise = () => {
  const data = useLoaderData<Omit<ExerciseData, "universes">>();

  const WebGui = lazy(async () => {
    return import(`exercises/${data.exercise_id}/frontend/WebGUI.tsx`).catch(
      (error) => {
        console.error("Component Failed Loading:", error);
        return { default: WebGUIPreview };
      }
    );
  });

  useEffect(() => {
    window.addEventListener("beforeunload", () => exitProject());

    return () => {
      window.removeEventListener("beforeunload", () => exitProject());
    };
  }, []);

  return (
    <>
      <ExerciseContainer
        multiLanguage={data.tags.includes("MULTILANGUAGE")}
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
