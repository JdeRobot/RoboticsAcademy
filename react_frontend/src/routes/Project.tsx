import React from "react";
import { Params, useLoaderData } from "react-router-dom";
import { lazy, Suspense } from "react";
import WebGUIPreview from "Components/visualizers/WebGUIPreview";
import ExerciseContainer from "Components/layouts/ExerciseContainer";
import { getProjectData } from "Api";
import WebGUILoading from "Components/visualizers/WebGUILoading";
import { ExerciseData } from "Types/exercises";

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
    return import(
      `exercises/${data.exercise_id}/react-components/WebGUI.js`
    ).catch((error) => {
      console.error("Component Failed Loading:", error);
      return { default: WebGUIPreview };
    });
  });

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

export default Exercise;
