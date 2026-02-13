import { lazy, Suspense } from "react";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { AcademyHeader } from "Components/headers";
import { StyledStubBackground } from "Styles/pages/Project.styles";
import { HomepageProvider } from "Contexts/HomepageContext";

const App = () => {
  const theme = useAcademyTheme();
  const AcademyContainer = lazy(
    () => import("Components/layouts/AcademyContainer")
  );

  return (
    <HomepageProvider>
      <AcademyHeader />
      <Suspense fallback={<StyledStubBackground bgColor={theme.palette.bg} />}>
        <AcademyContainer />
      </Suspense>
    </HomepageProvider>
  );
};

export default App;
