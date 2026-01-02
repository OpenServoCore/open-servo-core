//! AdcChannels derive macro implementation.
//!
//! Generates feature-gated ADC channel index constants from a struct definition.

use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::{
    parse::{Parse, ParseStream},
    Attribute, Data, DeriveInput, Fields, Ident, Lit, Meta, Token, Type,
};

/// Parsed `#[adc_channels(...)]` attribute.
struct AdcChannelsAttr {
    buffer_name: Ident,
    count_name: Ident,
}

impl Parse for AdcChannelsAttr {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut buffer_name = None;
        let mut count_name = None;

        while !input.is_empty() {
            let ident: Ident = input.parse()?;
            input.parse::<Token![=]>()?;

            match ident.to_string().as_str() {
                "buffer" => {
                    let lit: Lit = input.parse()?;
                    if let Lit::Str(lit_str) = lit {
                        buffer_name = Some(format_ident!("{}", lit_str.value()));
                    } else {
                        return Err(syn::Error::new_spanned(lit, "expected string literal"));
                    }
                }
                "count" => {
                    let lit: Lit = input.parse()?;
                    if let Lit::Str(lit_str) = lit {
                        count_name = Some(format_ident!("{}", lit_str.value()));
                    } else {
                        return Err(syn::Error::new_spanned(lit, "expected string literal"));
                    }
                }
                _ => return Err(syn::Error::new_spanned(ident, "unknown attribute")),
            }

            if input.peek(Token![,]) {
                input.parse::<Token![,]>()?;
            }
        }

        Ok(AdcChannelsAttr {
            buffer_name: buffer_name
                .ok_or_else(|| input.error("missing `buffer` attribute"))?,
            count_name: count_name
                .ok_or_else(|| input.error("missing `count` attribute"))?,
        })
    }
}

/// Parsed channel info from a struct field.
struct ChannelInfo {
    /// Field name (for accessor method).
    field_name: Ident,
    /// Const name from `#[channel(name = "...")]`.
    const_name: Ident,
    /// Feature name from `#[cfg(feature = "...")]`, if any.
    feature: Option<String>,
}

/// Parse `#[channel(name = "...")]` attribute.
fn parse_channel_attr(attrs: &[Attribute]) -> syn::Result<Option<Ident>> {
    for attr in attrs {
        if attr.path().is_ident("channel") {
            let meta = &attr.meta;
            if let Meta::List(list) = meta {
                // Parse: name = "CONST_NAME"
                let tokens = list.tokens.clone();
                let parsed: ChannelNameAttr = syn::parse2(tokens)?;
                return Ok(Some(parsed.name));
            }
        }
    }
    Ok(None)
}

/// Helper for parsing `name = "..."` inside #[channel(...)].
struct ChannelNameAttr {
    name: Ident,
}

impl Parse for ChannelNameAttr {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let ident: Ident = input.parse()?;
        if ident != "name" {
            return Err(syn::Error::new_spanned(ident, "expected `name`"));
        }
        input.parse::<Token![=]>()?;
        let lit: Lit = input.parse()?;
        if let Lit::Str(lit_str) = lit {
            Ok(ChannelNameAttr {
                name: format_ident!("{}", lit_str.value()),
            })
        } else {
            Err(syn::Error::new_spanned(lit, "expected string literal"))
        }
    }
}

/// Parse `#[cfg(feature = "...")]` attribute.
fn parse_cfg_feature(attrs: &[Attribute]) -> syn::Result<Option<String>> {
    for attr in attrs {
        if attr.path().is_ident("cfg") {
            let meta = &attr.meta;
            if let Meta::List(list) = meta {
                // Parse: feature = "feature-name"
                let tokens = list.tokens.clone();
                let parsed: CfgFeatureAttr = syn::parse2(tokens)?;
                return Ok(Some(parsed.feature));
            }
        }
    }
    Ok(None)
}

/// Helper for parsing `feature = "..."` inside #[cfg(...)].
struct CfgFeatureAttr {
    feature: String,
}

impl Parse for CfgFeatureAttr {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let ident: Ident = input.parse()?;
        if ident != "feature" {
            return Err(syn::Error::new_spanned(
                ident,
                "only `feature = \"...\"` is supported in #[cfg(...)]",
            ));
        }
        input.parse::<Token![=]>()?;
        let lit: Lit = input.parse()?;
        if let Lit::Str(lit_str) = lit {
            Ok(CfgFeatureAttr {
                feature: lit_str.value(),
            })
        } else {
            Err(syn::Error::new_spanned(lit, "expected string literal"))
        }
    }
}

/// Generate all 2^n combinations of feature presence/absence.
fn powerset(features: &[String]) -> Vec<Vec<String>> {
    let n = features.len();
    let count = 1usize << n;

    (0..count)
        .map(|mask| {
            features
                .iter()
                .enumerate()
                .filter(|(i, _)| mask & (1 << i) != 0)
                .map(|(_, f)| f.clone())
                .collect()
        })
        .collect()
}

/// Build #[cfg(...)] attribute for a specific combination.
///
/// - `own_feature`: This channel's feature requirement (if any).
/// - `prior_features`: All features from prior channels.
/// - `enabled`: Which prior features are enabled in this combination.
fn build_cfg(
    own_feature: Option<&str>,
    prior_features: &[String],
    enabled: &[String],
) -> TokenStream {
    let mut parts = Vec::new();

    // This channel's own feature requirement
    if let Some(feat) = own_feature {
        parts.push(quote!(feature = #feat));
    }

    // For each prior feature, specify enabled or not(enabled)
    for feat in prior_features {
        if enabled.contains(feat) {
            parts.push(quote!(feature = #feat));
        } else {
            parts.push(quote!(not(feature = #feat)));
        }
    }

    if parts.is_empty() {
        quote!()
    } else if parts.len() == 1 {
        let p = &parts[0];
        quote!(#[cfg(#p)])
    } else {
        quote!(#[cfg(all(#(#parts),*))])
    }
}

/// Calculate index offset for a combination of enabled features.
fn calculate_offset(channels: &[ChannelInfo], enabled: &[String]) -> usize {
    channels
        .iter()
        .filter(|ch| match &ch.feature {
            None => false, // Always-present channels are in base count
            Some(f) => enabled.contains(f),
        })
        .count()
}

/// Count always-present channels (no feature gate).
fn count_always_present(channels: &[ChannelInfo]) -> usize {
    channels.iter().filter(|ch| ch.feature.is_none()).count()
}

/// Get unique features from channels, preserving order.
fn unique_features(channels: &[ChannelInfo]) -> Vec<String> {
    let mut seen = Vec::new();
    for ch in channels {
        if let Some(f) = &ch.feature {
            if !seen.contains(f) {
                seen.push(f.clone());
            }
        }
    }
    seen
}

pub fn impl_adc_channels(input: &DeriveInput) -> syn::Result<TokenStream> {
    // Parse #[adc_channels(...)] attribute
    let attr = input
        .attrs
        .iter()
        .find(|a| a.path().is_ident("adc_channels"))
        .ok_or_else(|| syn::Error::new_spanned(input, "missing #[adc_channels(...)] attribute"))?
        .parse_args::<AdcChannelsAttr>()?;

    let buffer_name = &attr.buffer_name;
    let count_name = &attr.count_name;
    let struct_name = &input.ident;

    // Get struct fields
    let fields = match &input.data {
        Data::Struct(s) => match &s.fields {
            Fields::Named(named) => &named.named,
            _ => return Err(syn::Error::new_spanned(input, "expected named fields")),
        },
        _ => return Err(syn::Error::new_spanned(input, "expected struct")),
    };

    // Parse channel fields (skip non-channel fields like `buf`)
    let mut channels = Vec::new();
    let mut buf_field_name = None;

    for field in fields {
        let field_name = field.ident.clone().unwrap();

        // Check if this is the buffer field (type &'a [u16] or similar)
        if let Type::Reference(_) = &field.ty {
            buf_field_name = Some(field_name.clone());
            continue;
        }

        // Look for #[channel(...)] attribute
        if let Some(const_name) = parse_channel_attr(&field.attrs)? {
            let feature = parse_cfg_feature(&field.attrs)?;
            channels.push(ChannelInfo {
                field_name,
                const_name,
                feature,
            });
        }
    }

    if channels.is_empty() {
        return Err(syn::Error::new_spanned(
            input,
            "no channels defined (use #[channel(name = \"...\")])",
        ));
    }

    let buf_field = buf_field_name.ok_or_else(|| {
        syn::Error::new_spanned(input, "missing buffer field (e.g., `buf: &'a [u16]`)")
    })?;

    // Get all unique features
    let all_features = unique_features(&channels);

    // Generate ADC_CHANNEL_COUNT with nested cfg
    let base_count = count_always_present(&channels);
    let count_increments: Vec<TokenStream> = all_features
        .iter()
        .map(|feat| {
            // Count how many channels use this feature
            let n = channels.iter().filter(|ch| ch.feature.as_ref() == Some(feat)).count();
            quote! {
                #[cfg(feature = #feat)]
                let base = base + #n;
            }
        })
        .collect();

    let count_const = quote! {
        /// Number of ADC channels in the conversion sequence.
        pub const #count_name: usize = {
            let base = #base_count;
            #(#count_increments)*
            base
        };
    };

    // Generate buffer type alias
    let buffer_type = quote! {
        /// ADC DMA buffer type.
        pub type #buffer_name = [u16; #count_name];
    };

    // Generate mod idx with all cfg combinations
    let mut idx_consts = Vec::new();

    for (i, ch) in channels.iter().enumerate() {
        let const_name = &ch.const_name;

        // Get features from prior channels (for combination generation)
        let prior_features: Vec<String> = channels[..i]
            .iter()
            .filter_map(|c| c.feature.clone())
            .collect();
        let prior_unique: Vec<String> = {
            let mut seen = Vec::new();
            for f in prior_features {
                if !seen.contains(&f) {
                    seen.push(f);
                }
            }
            seen
        };

        // Base index = count of always-present channels before this one
        let base_idx = channels[..i].iter().filter(|c| c.feature.is_none()).count();

        if prior_unique.is_empty() {
            // No prior optional channels, simple case
            let cfg = build_cfg(ch.feature.as_deref(), &[], &[]);
            idx_consts.push(quote! {
                #cfg
                pub const #const_name: usize = #base_idx;
            });
        } else {
            // Generate all 2^n combinations of prior features
            let combos = powerset(&prior_unique);
            for combo in combos {
                let offset = calculate_offset(&channels[..i], &combo);
                let idx = base_idx + offset;
                let cfg = build_cfg(ch.feature.as_deref(), &prior_unique, &combo);

                idx_consts.push(quote! {
                    #cfg
                    pub const #const_name: usize = #idx;
                });
            }
        }
    }

    let idx_mod = quote! {
        /// Index constants for ADC buffer access.
        pub mod idx {
            #(#idx_consts)*
        }
    };

    // Generate accessor methods
    let mut accessor_methods = Vec::new();
    for ch in &channels {
        // Strip leading underscore from field name for method name
        let field_str = ch.field_name.to_string();
        let method_str = field_str.strip_prefix('_').unwrap_or(&field_str);
        let method_name = format_ident!("{}", method_str);
        let const_name = &ch.const_name;
        let cfg = if let Some(feat) = &ch.feature {
            quote!(#[cfg(feature = #feat)])
        } else {
            quote!()
        };

        accessor_methods.push(quote! {
            #cfg
            #[inline]
            pub fn #method_name(&self) -> u16 {
                self.#buf_field[idx::#const_name]
            }
        });
    }

    // Generate struct field initializers for new()
    let field_inits: Vec<TokenStream> = channels
        .iter()
        .map(|ch| {
            let name = &ch.field_name;
            if let Some(feat) = &ch.feature {
                quote! {
                    #[cfg(feature = #feat)]
                    #name: ()
                }
            } else {
                quote! { #name: () }
            }
        })
        .collect();

    // Generate impl block
    let impl_block = quote! {
        impl<'a> #struct_name<'a> {
            /// Create channels view from buffer reference.
            #[inline]
            pub fn new(buf: &'a #buffer_name) -> Self {
                Self {
                    #buf_field: buf,
                    #(#field_inits),*
                }
            }

            #(#accessor_methods)*
        }
    };

    Ok(quote! {
        #count_const
        #buffer_type
        #idx_mod
        #impl_block
    })
}
