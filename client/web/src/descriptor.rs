//! The descriptor surface: parse, select, code values. Wraps
//! `osc_client::descriptor` as two JS classes; field lists and values
//! cross as plain objects (`types`).

use osc_client::descriptor as desc;
use tsify::{Ts, Tsify};
use wasm_bindgen::prelude::*;

use crate::types::{Field, Selection, Value};

#[wasm_bindgen]
pub struct Descriptor(desc::Descriptor);

#[wasm_bindgen]
impl Descriptor {
    /// Parse format-2 descriptor JSON; throws on bad JSON or another format.
    pub fn parse(json: &str) -> Result<Descriptor, JsError> {
        Ok(Descriptor(desc::Descriptor::parse(json)?))
    }

    #[wasm_bindgen(getter)]
    pub fn model(&self) -> String {
        self.0.model.clone()
    }

    #[wasm_bindgen(getter, js_name = modelNumber)]
    pub fn model_number(&self) -> u16 {
        self.0.model_number
    }

    #[wasm_bindgen(getter, js_name = firmwareMajor)]
    pub fn firmware_major(&self) -> u8 {
        self.0.firmware_major
    }

    #[wasm_bindgen(getter, js_name = firmwareMinor)]
    pub fn firmware_minor(&self) -> u8 {
        self.0.firmware_minor
    }

    #[wasm_bindgen(getter, js_name = tableSize)]
    pub fn table_size(&self) -> u16 {
        self.0.table_size
    }

    pub fn fields(&self) -> Result<Vec<Ts<Field>>, JsError> {
        Ok(self
            .0
            .fields
            .iter()
            .map(|f| Field::from(f).into_ts())
            .collect::<Result<_, _>>()?)
    }

    pub fn decode(&self, name: &str, bytes: &[u8]) -> Result<Ts<Value>, JsError> {
        Ok(Value::from(desc::decode(self.field(name)?, bytes)?).into_ts()?)
    }

    pub fn encode(&self, name: &str, value: Ts<Value>) -> Result<Vec<u8>, JsError> {
        let value: Value = value.to_rust()?;
        Ok(desc::encode(self.field(name)?, &value.into())?)
    }
}

impl Descriptor {
    pub(crate) fn field(&self, name: &str) -> Result<&desc::Field, JsError> {
        self.0
            .field(name)
            .ok_or_else(|| JsError::new(&format!("no field {name} in {}", self.0.model)))
    }
}

#[wasm_bindgen]
#[derive(Default)]
pub struct Registry(desc::Registry);

#[wasm_bindgen]
impl Registry {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Registry {
        Registry::default()
    }

    /// Add a descriptor (copied), replacing one with the same model number
    /// and major.minor.
    pub fn push(&mut self, d: &Descriptor) {
        self.0.push(d.0.clone());
    }

    /// Pick the layout for a servo-reported (model, packed fw).
    pub fn select(&self, model: u16, fw: u16) -> Result<Ts<Selection>, JsError> {
        let sel = match self.0.select(model, fw) {
            desc::Selection::Exact(d) => Selection::Exact {
                index: self.index_of(d)?,
            },
            desc::Selection::OlderMinor(d) => Selection::OlderMinor {
                index: self.index_of(d)?,
            },
            desc::Selection::Incompatible { newest } => Selection::Incompatible { newest },
            desc::Selection::UnknownModel => Selection::UnknownModel,
        };
        Ok(sel.into_ts()?)
    }

    /// The descriptor at a `select` index (copied).
    pub fn at(&self, index: usize) -> Option<Descriptor> {
        self.0.iter().nth(index).cloned().map(Descriptor)
    }

    fn index_of(&self, d: &desc::Descriptor) -> Result<usize, JsError> {
        self.0
            .iter()
            .position(|e| std::ptr::eq(e, d))
            .ok_or_else(|| JsError::new("selected descriptor left the registry"))
    }
}
