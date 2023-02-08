import { useEffect, useRef } from "react";

export const useLoad = (fn) => {
  const cb = useRef(fn);

  useEffect(() => {
    const onLoad = cb.current;
    window.addEventListener("load", onLoad);
    return () => {
      window.removeEventListener("load", onLoad);
    };
  }, [cb]);
};
