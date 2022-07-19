export const setIframe = (iframe) => {
    iframe.setAttribute('src', "http://127.0.0.1:6080/vnc.html?resize=remote&autoconnect=true");
}

export const setIframeConsole = (console) => {
    console.setAttribute('src', "http://127.0.0.1:1108/vnc.html?resize=remote&autoconnect=true");
}