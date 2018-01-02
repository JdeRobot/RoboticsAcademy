'''
   Copyright (C) 1997-2017 JDERobot Developers Team

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Library General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.

   Authors : Okan Asik (asik.okan@gmail.com)

  '''
import yaml
class Generator(object):

    def __init__(self):
        pass

    def generateAndSaveCfgYaml(self, projectPath, projectName):
        cfgYaml = {}
        myInterfaces = {}
        interfaceIndexes = {}
        for cfg in self.config.getInterfaces():
            proxyName = None
            if 'proxyName' not in cfg:
                proxyName = cfg['interface']
            else:
                proxyName = cfg['proxyName']

            serverType = 0
            if cfg['serverType'] == 'ice':
                serverType = 1
            elif cfg['serverType'] == 'ros':
                serverType = 2

            myInterfaceData = {}
            myInterfaceData['Server'] = serverType
            myInterfaceData['Proxy'] = proxyName + ':default -h ' + cfg['ip'] + ' -p ' + str(cfg['port'])
            myInterfaceData['Topic'] = cfg['topic']
            myInterfaceData['Name'] = cfg['name']

            # TODO: remove this make sure that the interface lets user to enter these values
            if cfg['interface'] == 'Motors':
                myInterfaceData['maxW'] = 0.5
                myInterfaceData['maxV'] = 5.0
            elif cfg['interface'] == 'Camera':
                myInterfaceData['Format'] = 'RGB8'

            # idx = 0
            # if cfg['interface'] in interfaceIndexes:
            #     interfaceIndexes[cfg['interface']] += 1
            #     idx = interfaceIndexes[cfg['interface']]
            # else:
            #     interfaceIndexes[cfg['interface']] = idx

            # myInterfaces[cfg['interface'] + str(idx)] = myInterfaceData
            myInterfaces[cfg['name']] = myInterfaceData

        myInterfaces['NodeName'] = projectName

        cfgYaml[projectName] = myInterfaces
        with open(projectPath + '/' + projectName + '.yml', 'w') as outputFile:
            yaml.safe_dump(cfgYaml, outputFile, default_flow_style=False)


    def generateUserFunctions(self, functionsStr):
        for state in self.states:
            functionsStr.append(state.getFunctions())
            functionsStr.append('\n')
        return functionsStr

    def sanitizeVar(self, var):
        return var.replace(' ', '_')