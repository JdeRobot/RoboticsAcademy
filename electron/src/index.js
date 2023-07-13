const { app, BrowserWindow } = require('electron');
const path = require('path');
const { spawn } = require('child_process');

if (require('electron-squirrel-startup')) {
  app.quit();
}

const rootPath = path.resolve(__dirname, '../../')

let childProcess;

createDjangoServer = () => {
  const activateCommand = `source ${rootPath}/env/bin/activate && `;
  const command = `python3 ${rootPath}/manage.py runserver`;

  const fullCommand = activateCommand + command;
  const fullArgs = ['-c', fullCommand];

  childProcess = spawn('bash', fullArgs);
}

const killDjangoServer = () => {
  if (childProcess) {
    childProcess.kill();
  }
};

const createWindow = () => {
  const mainWindow = new BrowserWindow({
    width: 1280,
    height: 720,
    webPreferences: {
      nodeIntegration: true,
    },
  });
  mainWindow.loadURL('http://127.0.0.1:8000/');
};

app.on('ready', () => {
  createDjangoServer()
  setTimeout(() => {
    createWindow()
  }, 5000);
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
    killDjangoServer()
  }
});

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) {
    createWindow();
  }
});
