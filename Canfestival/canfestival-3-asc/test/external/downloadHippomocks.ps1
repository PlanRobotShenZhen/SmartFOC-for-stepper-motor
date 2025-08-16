$scriptFolder = $PSScriptRoot

Invoke-WebRequest https://raw.githubusercontent.com/dascandy/hippomocks/master/HippoMocks/comsupport.h -OutFile $scriptFolder/comsupport.h
Invoke-WebRequest https://raw.githubusercontent.com/dascandy/hippomocks/master/HippoMocks/hippomocks.h -OutFile $scriptFolder/hippomocks.h
