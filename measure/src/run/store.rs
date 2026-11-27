//! Where a run's files live: the CLI's file system, the browser's memory.
//! The run names every file by its path and opens none itself; a
//! [`Store`] does, and [`At`] is a run folder in one, the [`Sink`] the
//! steps write through.

use std::path::{Path, PathBuf};

use crate::sink::{MemSink, Sink, SinkError};

/// The files a run reads and writes. An error is the cause alone, as the
/// host words it (`No such file or directory (os error 2)`): the caller
/// names the path.
pub trait Store {
    /// The bytes of the file at `path`.
    fn read_file(&mut self, path: &Path) -> Result<Vec<u8>, String>;
    /// `bytes`, whole, as the file at `path`, over one that is there; its
    /// folder is there already.
    fn write_file(&mut self, path: &Path, bytes: &[u8]) -> Result<(), String>;
    /// The folder `path` and every folder above it.
    fn make_dirs(&mut self, path: &Path) -> Result<(), String>;
    /// Whether a file or a folder is at `path`.
    fn exists(&self, path: &Path) -> bool;
    /// The names in the folder `path`, each with whether it is a folder,
    /// in no order.
    fn entries(&self, path: &Path) -> Result<Vec<(String, bool)>, String>;
    /// The one name of the folder or file at `path`, links and `..`
    /// resolved, so that two names of one folder compare equal.
    fn canonical(&self, path: &Path) -> Result<PathBuf, String>;
}

/// The run folder `root` in a store: a file a run kept is never written
/// over.
pub struct At<'a> {
    pub files: &'a mut dyn Store,
    pub root: PathBuf,
}

impl Sink for At<'_> {
    fn write(&mut self, path: &str, bytes: &[u8]) -> Result<(), SinkError> {
        let p = self.root.join(path);
        if self.files.exists(&p) {
            return Err(SinkError(format!(
                "{} exists: a run never writes over its files",
                p.display()
            )));
        }
        if let Some(dir) = p.parent() {
            self.files
                .make_dirs(dir)
                .map_err(|e| SinkError(format!("{}: {e}", dir.display())))?;
        }
        self.files
            .write_file(&p, bytes)
            .map_err(|e| SinkError(format!("{}: {e}", p.display())))
    }

    fn read(&mut self, path: &str) -> Result<Vec<u8>, SinkError> {
        let p = self.root.join(path);
        self.files
            .read_file(&p)
            .map_err(|e| SinkError(format!("{}: {e}", p.display())))
    }

    fn shown(&self, path: &str) -> String {
        self.root.join(path).display().to_string()
    }
}

/// A memory sink as a whole store: its keys are the files' full paths, and
/// a folder is there while a file is below it.
impl Store for MemSink {
    fn read_file(&mut self, path: &Path) -> Result<Vec<u8>, String> {
        self.files
            .get(&key(path))
            .cloned()
            .ok_or_else(|| "no such file".to_string())
    }

    fn write_file(&mut self, path: &Path, bytes: &[u8]) -> Result<(), String> {
        self.files.insert(key(path), bytes.to_vec());
        Ok(())
    }

    fn make_dirs(&mut self, _: &Path) -> Result<(), String> {
        Ok(())
    }

    fn exists(&self, path: &Path) -> bool {
        let k = key(path);
        self.files.contains_key(&k) || self.files.keys().any(|f| below(f, &k).is_some())
    }

    fn entries(&self, path: &Path) -> Result<Vec<(String, bool)>, String> {
        let k = key(path);
        let mut out: Vec<(String, bool)> = Vec::new();
        for f in self.files.keys() {
            let Some(rest) = below(f, &k) else { continue };
            let e = match rest.split_once('/') {
                Some((name, _)) => (name.to_string(), true),
                None => (rest.to_string(), false),
            };
            if !out.contains(&e) {
                out.push(e);
            }
        }
        if out.is_empty() {
            return Err("no such folder".into());
        }
        Ok(out)
    }

    fn canonical(&self, path: &Path) -> Result<PathBuf, String> {
        Ok(path.to_path_buf())
    }
}

fn key(path: &Path) -> String {
    path.display().to_string()
}

/// `file`'s path below the folder `dir`, where it is below it.
fn below<'a>(file: &'a str, dir: &str) -> Option<&'a str> {
    let rest = file.strip_prefix(dir)?;
    if dir.ends_with('/') {
        Some(rest)
    } else {
        rest.strip_prefix('/')
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A run folder in memory refuses to write over a file, reads back what
    /// it wrote, and lists its folders as a file system does.
    #[test]
    fn a_run_folder_in_memory_keeps_its_files() {
        let mut m = MemSink::default();
        let mut at = At {
            files: &mut m,
            root: PathBuf::from("/runs/uid/t"),
        };
        at.write("run.json", b"{}").expect("write");
        at.write("captures/rest/01.csv.gz", b"x").expect("write");
        let e = at.write("run.json", b"{}").expect_err("refused");
        assert_eq!(
            e.0,
            "/runs/uid/t/run.json exists: a run never writes over its files"
        );
        assert_eq!(at.read("run.json").expect("read"), b"{}");
        assert_eq!(
            at.read("none.json").expect_err("none").0,
            "/runs/uid/t/none.json: no such file"
        );
        assert_eq!(at.shown("run.json"), "/runs/uid/t/run.json");
        assert!(m.exists(Path::new("/runs/uid")) && !m.exists(Path::new("/runs/ui")));
        let mut e = m.entries(Path::new("/runs/uid/t")).expect("entries");
        e.sort();
        assert_eq!(
            e,
            [
                ("captures".to_string(), true),
                ("run.json".to_string(), false)
            ]
        );
        assert!(m.entries(Path::new("/elsewhere")).is_err());
    }
}
