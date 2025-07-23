import * as React from "react";

function ForumView: React.FC {
  const forumRef = React.useRef<HTMLIFrameElement>(null);
  const [iFrameHeight, setIframeHeight] = React.useState("1000px");
  const loadIframeHeight = () => {
    setIframeHeight(
      forumRef.current.contentWindow.document.body.scrollHeight + "px"
    );
  };
  return (
    <iframe
      src={"https://forum.unibotics.org/"}
      id="forum-view"
      width={"100%"}
      ref={forumRef}
      height={iFrameHeight}
      onLoad={loadIframeHeight}
      style={{
        width: "100%",
        overflow: "auto",
      }}
    />
  );
}

export default ForumView;
