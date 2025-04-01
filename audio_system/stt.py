from pydub import AudioSegment

# Converts audio file to wav format
def audio_file_converter(audio_file):
    if not isinstance(audio_file, str):
        raise ValueError("Audio file must be a string")

    # Load the audio file
    audio = AudioSegment.from_file(audio_file)

    # Define the output file path
    output_file = audio_file.rsplit('.', 1)[0] + '.wav'

    # Export the audio file as WAV
    audio.export(output_file, format='wav')

    return output_file


    