pub struct GenerationalArena<T> {
    entries:   Vec<ArenaEntry<T>>,
    free_list: Vec<u32>,             
    len:       usize,                
}

enum ArenaEntry<T> {
    Occupied { generation: u32, value: T },
    Free     { generation: u32 },    
}

impl<T> GenerationalArena<T> {
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
            free_list: Vec::new(),
            len: 0,
        }
    }

    pub fn with_capacity(cap: usize) -> Self {
        Self {
            entries: Vec::with_capacity(cap),
            free_list: Vec::with_capacity(cap),
            len: 0,
        }
    }

    pub fn insert(&mut self, value: T) -> (u32, u32) {
        self.len += 1;


        if let Some(slot_index) = self.free_list.pop() {
            let idx = slot_index as usize;
            

            let next_gen = match &self.entries[idx] {
                ArenaEntry::Free { generation } => generation.wrapping_add(1),
                ArenaEntry::Occupied { generation, .. } => generation.wrapping_add(1),
            };

            self.entries[idx] = ArenaEntry::Occupied {
                generation: next_gen,
                value,
            };
            (slot_index, next_gen)
        } else {

            let slot_index = self.entries.len() as u32;
            let generation = 0;
            
            self.entries.push(ArenaEntry::Occupied {
                generation,
                value,
            });
            (slot_index, generation)
        }
    }

    pub fn remove(&mut self, slot: u32, gen: u32) -> Option<T> {
        if slot as usize >= self.entries.len() {
            return None;
        }
        
        match &self.entries[slot as usize] {
            ArenaEntry::Free { .. } => return None,
            ArenaEntry::Occupied { generation, .. } if *generation != gen => return None,
            _ => {} 
        }

        let old_entry = std::mem::replace(
            &mut self.entries[slot as usize], 
            ArenaEntry::Free { generation: 0 }
        );

        if let ArenaEntry::Occupied { generation, value } = old_entry {
            let next_generation = generation.wrapping_add(1);
            
            self.entries[slot as usize] = ArenaEntry::Free { generation: next_generation };
            
            self.free_list.push(slot); 
            self.len -= 1; 
            
            Some(value)
        } else {
            None
        }
    }

    pub fn get(&self, slot: u32, gen: u32) -> Option<&T> {
        if slot as usize >= self.entries.len() {
            return None;
        }
        
        match &self.entries[slot as usize] {
            ArenaEntry::Occupied { generation, value } if *generation == gen => Some(value),
            _ => None,
        }
    }

    pub fn get_mut(&mut self, slot: u32, gen: u32) -> Option<&mut T> {
        if slot as usize >= self.entries.len() {
            return None;
        }
        
        match &mut self.entries[slot as usize] {
            ArenaEntry::Occupied { generation, value } if *generation == gen => Some(value),
            _ => None,
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.entries.iter().filter_map(|entry| {
            if let ArenaEntry::Occupied { value, .. } = entry {
                Some(value)
            } else {
                None
            }
        })
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.entries.iter_mut().filter_map(|entry| {
            if let ArenaEntry::Occupied { value, .. } = entry {
                Some(value)
            } else {
                None
            }
        })
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn arena_insert_and_get() {




        let mut arena = super::GenerationalArena::new();
        let (slot, gen) = arena.insert(42u32);
        assert_eq!(arena.get(slot, gen), Some(&42));
        assert_eq!(arena.len(), 1);

    }

    #[test]
    fn arena_remove_then_get_returns_none() {




        let mut arena = super::GenerationalArena::new();
        let (slot, gen) = arena.insert(99);
        arena.remove(slot, gen);
        assert_eq!(arena.get(slot, gen), None);
        assert_eq!(arena.len(), 0);

    }

    #[test]
    fn arena_stale_handle_rejected() {







        let mut arena = super::GenerationalArena::new();
        let (slot, gen0) = arena.insert("first");
        arena.remove(slot, gen0);
        let (slot2, gen1) = arena.insert("second");
        assert_eq!(slot, slot2);
        assert_eq!(arena.get(slot, gen0), None);
        assert_eq!(arena.get(slot, gen1), Some(&"second"));

    }

    #[test]
    fn arena_iter_skips_free_slots() {





        let mut arena = super::GenerationalArena::new();
        let (_s0, _g0) = arena.insert("A");
        let (s1, g1) = arena.insert("B");
        let (_s2, _g2) = arena.insert("C");
        arena.remove(s1, g1);
        let items: Vec<_> = arena.iter().collect();
        assert_eq!(items.len(), 2);
        assert!(items.contains(&&"A"));
        assert!(items.contains(&&"C"));
        
    }
}
