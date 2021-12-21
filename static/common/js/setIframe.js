function setIframe(){
    var iframe = document.querySelector('#iframe');
    iframe.setAttribute('src', "http://127.0.0.1:6080/vnc.html?resize=remote&autoconnect=true");
};

function setIframeConsole(){
    var console = document.querySelector('#console-vnc');
    console.setAttribute('src', "http://127.0.0.1:1108/vnc.html?resize=remote&autoconnect=true");
}