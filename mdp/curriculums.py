import torch
import random


class PianistCurriculum:
    NOTE_OFFSETS_Y = (0.0, 0.0235, 0.047, 0.0705, 0.094, 0.1175, 0.141)

    @staticmethod
    def sequenced_note_curriculum(env, env_ids):
        return sequenced_note_curriculum(env, env_ids)


def sequenced_note_curriculum(env, env_ids):
    # The 7 notes of C Major Scale (relative coords to Middle C)
    # [C4, D4, E4, F4, G4, A4, B4]
    notes_x_offsets = torch.tensor([0.0, 0.0235, 0.047, 0.0705, 0.094, 0.1175, 0.141], device=env.device)

    # Every 500 steps, pick a new target from ONLY these 7 notes
    num_notes = len(notes_x_offsets)
    indices = torch.randint(0, num_notes, (len(env_ids),), device=env.device)

    # Update the environment's current_midi_goal
    # Assuming base_c4_pos is [0.5, 0.0, 0.2]
    new_goals = env.base_c4_pos[env_ids].clone()
    new_goals[:, 1] += notes_x_offsets[indices]  # Offset along the keyboard Y-axis

    env.current_target_pos[env_ids] = new_goals
    env.current_note_indices[env_ids] = indices
    env.current_midi_goal[env_ids] = 0.0
    env.current_midi_goal[env_ids, indices] = 1.0
