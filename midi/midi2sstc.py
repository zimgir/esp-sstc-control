
# Mostly generated with ChatGPT using sequential prompts

import argparse
import mido

DEFAULT_FILE_PATH="midi\\russtc.mid"

def midi_to_freq(midi_note):
    """
    Converts a MIDI note number to a frequency in Hz.
    """
    return 440 * (2 ** ((midi_note - 69) / 12))


def parse_midi_file(midi_file_path):
    """
    Parses a MIDI file and returns a list of lists, where each sublist contains
    a track's note frequencies and their durations in milliseconds, including "silence" notes.
    """
    mid = mido.MidiFile(midi_file_path)

    tempo = mido.bpm2tempo(120)  # Set default tempo to 120 bpm

    active_note_start_time = {}

    note_tracks = [[] for _ in range(len(mid.tracks))] 

    for i, track in enumerate(mid.tracks):
        for message in track:
            if message.type == 'note_on' and message.velocity != 0:
                # note_on message.time is tick difference from previous event
                # in terms of notes this can be parsed as duration of silence
                # from previous note_off event
                active_note_start_time[message.note] = message.time


            elif message.type == 'note_off' or (message.type == 'note_on' and message.velocity == 0):
                # note_of message.time is tick difference from previous event
                # in terms of notes this can be parsed as duration of note
                # from previous note_on event
                note_number = message.note  # MIDI note number
                
                if note_number in active_note_start_time:
    
                    # Add "silence" note if there was a gap since the last note
                    if active_note_start_time[note_number] > 0:
                        silence_duration_ticks = active_note_start_time[note_number]
                        silence_duration_sec = mido.tick2second(silence_duration_ticks, mid.ticks_per_beat, tempo)
                        silence_duration_ms = silence_duration_sec * 1000
                        note_tracks[i].append((0, silence_duration_ms))

                    # Add note to list
                    note_duration_ticks = message.time
                    note_duration_sec = mido.tick2second(note_duration_ticks, mid.ticks_per_beat, tempo)
                    note_duration_ms = note_duration_sec * 1000
                    note_frequency = midi_to_freq(note_number)
                    note_tracks[i].append((note_frequency, note_duration_ms))

                    # remove from active notes
                    del active_note_start_time[note_number]

            elif message.type == 'set_tempo':
                tempo = message.tempo

    
    return note_tracks

def main():
    parser = argparse.ArgumentParser(description='Parse a MIDI file to a list of frequencies and durations')
    parser.add_argument('--midi_file_path', type=str, help='Path to the MIDI file to parse')
    args = parser.parse_args()
    
    if args.midi_file_path is None:
        # Set a default MIDI file path if the argument is not provided
        args.midi_file_path = DEFAULT_FILE_PATH

    note_tracks = parse_midi_file(args.midi_file_path)

    for i, track in enumerate(note_tracks):

        min_freq = float('inf')
        max_freq = float('-inf')
        min_dur = float('inf')
        max_dur = float('-inf')

        print("typedef struct {\n   uint16_t freq;\n   uint16_t dur_ms;\n} program_note_t;\n")

        print(f"const program_note_t track_{i}[] = {{")
        for j, note in enumerate(track):
            freq = note[0]
            dur = note[1]

            if freq < min_freq:
                min_freq = freq
            if freq > max_freq:
                max_freq = freq
            if dur < min_dur:
                min_dur = dur
            if dur > max_dur:
                max_dur = dur

            print(f"   {{ {int(freq):5d}, {int(dur):5d}, }} , // note {j}")
        print("};")

        print(f'\n\nFrequency: {min_freq:.2f} ~ {max_freq:.2f} Hz')
        print(f'Duration: {min_dur:.2f} ~ {max_dur:.2f} ms\n\n')




if __name__ == '__main__':
    main()

