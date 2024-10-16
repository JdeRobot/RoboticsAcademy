from rest_framework import viewsets
from rest_framework import permissions

# all import for pylint and black
from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status
import black
import tempfile
import subprocess
import json
import os

# serializer
from academy.academy_rest_api.serializers.exercises import CodeFormatSerializer
from academy.academy_rest_api.serializers.exercises import CodeAnalysisSerializer


# Create your views here.
from academy.academy_rest_api.serializers.exercises import ExerciseSerializer
from exercises.models import Exercise


class ExerciseViewSet(viewsets.ModelViewSet):
    queryset = Exercise.objects.all()
    serializer_class = ExerciseSerializer
    permission_classes = [permissions.IsAuthenticatedOrReadOnly]





# Code format with Black
class FormatCode(APIView):
    def post(self, request, *args, **kwargs):
        serializer = CodeFormatSerializer(data=request.data)
        
        if serializer.is_valid():
            code = serializer.validated_data['code']
            try:
                # Format the code with Black
                formatted_code = black.format_str(code, mode=black.Mode())
                return Response({'formatted_code': formatted_code}, status=status.HTTP_200_OK)
            except Exception as e:
                return Response({'error': str(e)}, status=status.HTTP_400_BAD_REQUEST)
        
        return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)



# Code Analysis with Pylint 
class CodeAnalysis(APIView):
    def post(self, request, *args, **kwargs):
        serializer = CodeAnalysisSerializer(data=request.data)
        
        # validate data
        if serializer.is_valid():
            # Extract the validated data
            code_string = serializer.validated_data.get('code')
            disable_error_ids = serializer.validated_data.get('disable_errors')

            # if code string is empty
            if not code_string:
                return Response({"pylint_errors": "No code provided."}, status=status.HTTP_400_BAD_REQUEST)


            # Save the code string to a temporary file
            with tempfile.NamedTemporaryFile(suffix=".py", delete=False) as temp_file:
                temp_file.write(code_string.encode('utf-8'))
                temp_file_path = temp_file.name
                
            
            # terminal command
            command = ['pylint', '--output-format=json',] + [temp_file_path]
            # '--extension-pkg-whitelist=cv2'
            
            # Add the disable option for specific error IDs
            if disable_error_ids:
                disable_str = ','.join(disable_error_ids)
                command.append(f'--disable={disable_str}')
            
            # run the command
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            # Decode the results
            pylint_output = result.stdout.decode('utf-8')
            pylint_errors = result.stderr.decode('utf-8')
            
            # Parse the JSON output if pylint output is not empty
            try:
                pylint_json = json.loads(pylint_output) if pylint_output else []
            except json.JSONDecodeError as e:
                pylint_json = {"error": f"Failed to parse JSON: {str(e)}"}

            
            # Clean up the temporary file after Pylint run
            if os.path.exists(temp_file_path):
                os.remove(temp_file_path)
            
            if pylint_errors:
                return Response({"pylint_output": pylint_json, "pylint_errors": pylint_errors}, status=status.HTTP_500_INTERNAL_SERVER_ERROR)
            
            return Response({"pylint_output": pylint_json}, status=status.HTTP_200_OK)
    