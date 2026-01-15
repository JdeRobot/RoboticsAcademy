import React, { useEffect, useRef, useState } from "react";
import { useParams } from "react-router-dom";
import { lazy, Suspense } from "react";
import WebGUIPreview from "Components/visualizers/WebGUIPreview";
import ExerciseContainer from "Components/layouts/ExerciseContainer";
import { getProjectData } from "Api";
import { HomepageProvider } from "Contexts/HomepageContext";
import { AcademyHeader } from "Components/headers";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { StyledStubBackground } from "Styles/pages/Project.styles";

const Exercise = () => {
  const { proj_id } = useParams();
  const theme = useAcademyTheme();
  const infoRef = useRef<any>(null);
  const [loading, setLoading] = useState<boolean>(true);

  const getInfo = async () => {
    const newInfo = await getProjectData(proj_id);
    infoRef.current = newInfo;
    setLoading(false);
  };

  useEffect(() => {
    getInfo();
  }, []);

  const WebGui = lazy(() =>
    import(`exercises/${proj_id}/react-components/WebGUI.js`).catch((error) => {
      console.error("Component Failed Loading:", error);
      return { default: WebGUIPreview };
    })
  );

  if (infoRef.current === null || proj_id === undefined || loading) {
    return (
      <HomepageProvider>
        <AcademyHeader />
        <StyledStubBackground bgColor={theme.palette.bg}/>
      </HomepageProvider>
    );
  }

  return (
    <>
      <ExerciseContainer
        hasDLModel={infoRef.current.tags.includes("Deep Learning")}
        multiLanguage={infoRef.current.tags.includes("MULTILANGUAGE")}
        project={proj_id}
        name={infoRef.current.name}
        tools={infoRef.current.tools}
        url={infoRef.current.url !== "" ? infoRef.current.url : undefined}
      >
        <Suspense fallback={<Loading />}>
          <WebGui />
        </Suspense>
      </ExerciseContainer>
    </>
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
