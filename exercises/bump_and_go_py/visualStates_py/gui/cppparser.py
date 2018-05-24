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

class CPPParser():

    def __init__(self):
        pass

    @staticmethod
    def parseFunctions(funcStr):
        returnTypes = []
        funcNames = []
        codes = []
        funcExists = True
        while funcExists and len(funcStr) > 0 and funcStr.index('{') >= 0:
            funcStr = funcStr.strip()
            funcStartIndex = funcStr.index('{')
            funcSignature = funcStr[0:funcStartIndex].strip()
            returnType = funcSignature[0:funcSignature.index(' ')].strip()
            returnTypes.append(returnType)
            funcName = funcSignature[funcSignature.index(' '):].strip()
            funcNames.append(funcName)
            curlyCounter = 0
            firstCurlyFound = False
            firstCurlyIndex = None
            lastCurlyIndex = None
            for i, ch in enumerate(funcStr):
                if ch == '{':
                    curlyCounter += 1
                    if not firstCurlyFound:
                        firstCurlyFound = True
                        firstCurlyIndex = i
                elif ch == '}':
                    curlyCounter -= 1

                if curlyCounter == 0 and firstCurlyFound:
                    lastCurlyIndex = i
                    break
            # print(firstCurlyIndex)
            # print(lastCurlyIndex)
            # print(funcStr[firstCurlyIndex:lastCurlyIndex+1])
            codes.append(funcStr[firstCurlyIndex:lastCurlyIndex+1])
            funcExists = False

            # check whether there is any other function
            funcStr = funcStr[lastCurlyIndex+1:].strip()
            if len(funcStr) > 0 and funcStr.index('{') >= 0:
                funcExists = True

        # return returnType, funcName
        # print(returnType)
        # print(funcName)
        return returnTypes, funcNames, codes


    @staticmethod
    def parseVariables(variableStr):
        types = []
        varNames = []
        initialValues = []
        variableStr = variableStr.strip()
        variableLines = variableStr.split(';')
        for varLine in variableLines:
            varLine = varLine.strip()
            if len(varLine) == 0:
                continue

            varType = varLine[0:varLine.find(' ')]
            varName = None
            initalValue = None
            if varLine.find('=') >= 0:
                # if there is initial value
                varName = varLine[varLine.find(' ')+1:varLine.find('=')].strip()
                initialValue = varLine[varLine.find('=')+1:].strip()
            else:
                varName = varLine[varLine.find(' ')+1:].strip()

            types.append(varType)
            varNames.append(varName)
            initialValues.append(initialValue)

        return types, varNames, initialValues


if __name__ == '__main__':
    sampleCode = '''
  void myFunction(int a) {
    int c;
    c = a * 2;
}

void myF2 ( int b ){
    int c,d;
    c = 10;
    d = 12;
    a = c*d*b;
    if ( a == 2) {
        b = 3;
    }
    return a;
    }
    
    
    
    int myfunc3() {
        int a = 12;
        int b = 324;
        int c = 0;
        c = a + b;
        for (int i = 0; i < 10; i++) {
            c = a + b;
        }
    }
'''

    # returnTypes, funcNames, codes = CPPParser.parseFunctions(sampleCode)
    # for i in range(len(returnTypes)):
    #     print(returnTypes[i])
    #     print(funcNames[i])
    #     print(codes[i])
    # print(returnType, funcName)

    sampleVariables = '''
    int a = 12; int b = 23;
    float myVar; float myVar2 = 12.2;
    '''

    types, varNames, initialValues = CPPParser.parseVariables(sampleVariables)
    for i in range(len(types)):
        print(types[i])
        print(varNames[i])
        print(initialValues[i])

