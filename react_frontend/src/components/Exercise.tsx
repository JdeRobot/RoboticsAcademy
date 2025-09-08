import { ErrorProvider } from "jderobot-ide-interface";
import { AcademyThemeProvider } from "Contexts/AcademyThemeContext";
import ExerciseContainer from "./layouts/ExerciseContainer";
import { ExerciseData } from "Types/exercises";
import { lazy, Suspense } from "react";
import WebGUIPreview from "./visualizers/WebGUIPreview";

const Exercise = () => {
  const exerciseData = document.getElementById("exercise-data");
  console.log(exerciseData);

  let config: ExerciseData | undefined = undefined;
  if (exerciseData !== null) {
    config = JSON.parse(exerciseData.textContent);
  }

  const project = config?.name || "";
  const exercise_id = config?.exercise_id || "";
  const tools = config?.tools || [];
  const tags = config?.tags || [];
  var urlRaw = config?.url || "";
  const url = urlRaw !== "" ? urlRaw : undefined;

  const WebGui = lazy(() =>
    import(`exercises/${exercise_id}/react-components/WebGUI.js`).catch(
      (error) => {
        console.error("Component Failed Loading:", error);
        return { default: WebGUIPreview };
      }
    )
  );

  return (
    <ErrorProvider>
      <AcademyThemeProvider>
        <ExerciseContainer
          hasDLModel={tags.includes("Deep Learning")}
          project={project}
          tools={tools}
          url={url}
        >
          <Suspense fallback={<Loading />}>
            <WebGui />
          </Suspense>
        </ExerciseContainer>
      </AcademyThemeProvider>
    </ErrorProvider>
  );
};

function Loading() {
  return (
    <p>
      <i>Loading...</i>
    </p>
  );
}

export default Exercise;
