import React from "react";
import { Params, useLoaderData } from "react-router-dom";
import { lazy, Suspense } from "react";
import WebGUIPreview from "Components/visualizers/WebGUIPreview";
import ExerciseContainer from "Components/layouts/ExerciseContainer";
import { getProjectData } from "Api";
import WebGUILoading from "Components/visualizers/WebGUILoading";
import { ExerciseData } from "Types/exercises";
import { StyledExerciseContainer } from "Styles/layouts/ExerciseContainer.styles";
import { useTabLock } from "../hooks/useTabLock";

export const loader = async ({
  params,
}: {
  params: Params<string>;
}): Promise<Omit<ExerciseData, "universes">> => {
  return await getProjectData(params.proj_id);
};

const Exercise = () => {
  const data = useLoaderData<Omit<ExerciseData, "universes">>();
  const { isLocked, isLoading } = useTabLock(data.exercise_id);

  const WebGui = lazy(async () => {
    return import(
      `exercises/${data.exercise_id}/react-components/WebGUI.js`
    ).catch((error) => {
      console.error("Component Failed Loading:", error);
      return { default: WebGUIPreview };
    });
  });

  if (isLoading) {
    return <WebGUILoading />;
  }

  if (isLocked) {
    return (
      <StyledExerciseContainer
        style={{
          alignItems: "center",
          justifyContent: "center",
          height: "100vh",
          backgroundColor: "#282c34",
          color: "white",
        }}
      >
        <h1>Exercise already open</h1>
        <p>
          This exercise is already open in another tab. Please use that one or
          close it to continue.
        </p>
        <button
          onClick={() => window.location.reload()}
          style={{
            marginTop: "20px",
            padding: "10px 20px",
            fontSize: "16px",
            cursor: "pointer",
          }}
        >
          Retry
        </button>
      </StyledExerciseContainer>
    );
  }

  return (
    <>
      <ExerciseContainer
        hasDLModel={data.tags.includes("Deep Learning")}
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

export default Exercise;
